"""Thu âm và giải mã trong tiến trình riêng; không chạy ROS trong worker."""
import os
import queue
import signal
import threading
import time
from collections import deque


def capture_microphone(device, rate, output, shutdown, parent_pid, audio_gain=1.8):
    """Luồng thu âm chỉ đọc mẫu, chỉnh gain rồi đưa vào hàng đợi giới hạn."""
    import sounddevice as sd
    import numpy as np
    gap = False
    try:
        with sd.RawInputStream(samplerate=rate, blocksize=0, device=device,
                               channels=1, dtype='int16', latency='high') as stream:
            # Khối 100 ms giảm số lần chuyển dữ liệu và gọi bộ giải mã.
            frames = max(1, rate // 10)
            while not shutdown.is_set() and os.getppid() == parent_pid:
                data, overflow = stream.read(frames)
                captured, wall = time.monotonic(), time.time()
                samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                samples -= samples.mean()
                peak = float(np.max(np.abs(samples)))
                gain = min(audio_gain, 30000.0/peak) if peak else audio_gain
                data = np.clip(samples*gain, -30000, 30000).astype(np.int16).tobytes()
                gap = gap or bool(overflow)
                try:
                    output.put_nowait(('audio', data, captured, wall, gap))
                    gap = False
                except queue.Full:
                    # Đánh dấu phần âm thanh đã mất, không ghép hai đoạn rời thành câu.
                    gap = True
    except Exception as error:
        try:
            output.put(('error', str(error), time.monotonic(), time.time(), True), timeout=0.2)
        except queue.Full:
            pass


def recognize_microphone(model_path, device_id, sample_rate, audio_gain,
                         messages, shutdown, parent_pid, room_vocabulary='', use_hotwords=False):
    """Thu/VAD và giải mã ở hai luồng; không bỏ câu mới khi Whisper đang chạy."""
    import audioop
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    messages.cancel_join_thread()
    capture = decoder = None

    def emit(kind, data):
        try:
            messages.put((kind, data), timeout=0.1)
        except queue.Full:
            pass

    try:
        import numpy as np
        import sounddevice as sd
        import webrtcvad
        from faster_whisper import WhisperModel
        device = None if device_id < 0 else device_id
        info = sd.query_devices(device, 'input')
        rate = sample_rate
        try:
            sd.check_input_settings(device=device, samplerate=rate, channels=1, dtype='int16')
        except sd.PortAudioError:
            rate = int(info['default_samplerate'])
        # Hai luồng CPU để còn tài nguyên cho Gazebo và Nav2.
        model = WhisperModel(model_path, device='cpu', compute_type='int8',
                             cpu_threads=2, num_workers=1, local_files_only=True)
        if shutdown.is_set():
            return
        audio = queue.Queue(maxsize=20)
        capture = threading.Thread(target=capture_microphone,
            args=(device, rate, audio, shutdown, parent_pid, audio_gain), daemon=True)
        capture.start()
        # Một luồng giải mã riêng: vòng thu/VAD bên dưới luôn tiếp tục nghe.
        # Giới hạn một câu chờ để không xếp hàng nhiều yêu cầu cũ.
        jobs = queue.Queue(maxsize=1)

        def decode_sentences():
            while not shutdown.is_set() and os.getppid() == parent_pid:
                try:
                    waveform, started_at, observed_at = jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    emit('state', 'Whisper đang giải mã — microphone vẫn đang nghe')
                    # Mặc định không gợi ý từ để đánh giá khả năng nghe của model.
                    # Có thể bật lại bằng tham số launch khi cần.
                    vocabulary = ('Dừng lại, dừng robot, hủy lệnh, stop, '+room_vocabulary
                                  if use_hotwords else None)
                    segments, _ = model.transcribe(waveform, language='vi', task='transcribe',
                        beam_size=5, temperature=0, condition_on_previous_text=False,
                        hotwords=vocabulary, vad_filter=True)
                    segments = list(segments)
                    text = ''.join(part.text for part in segments).strip()
                    reliable = bool(segments) and all(part.avg_logprob >= -1.0 and
                        part.no_speech_prob < 0.7 for part in segments)
                    if shutdown.is_set():
                        return
                    if text and reliable:
                        emit('final', {'text': text, 'final': True,
                            'started_at': started_at, 'observed_at': observed_at})
                    else:
                        emit('rejected', 'Chưa nghe rõ; hãy nói lại, hoặc dùng nút DỪNG ROBOT.')
                    emit('state', 'Microphone đang nghe tiếng Việt')
                except Exception as error:
                    emit('fatal', 'Lỗi giải mã: '+str(error))
                    shutdown.set()
                    return

        decoder = threading.Thread(target=decode_sentences, daemon=True)
        decoder.start()
        emit('ready', {'device': str(info['name']), 'sample_rate': rate})
        vad = webrtcvad.Vad(2)
        # Mỗi khung VAD dài 20 ms: 16000 * 0.02 * 2 byte = 640 byte.
        frame_bytes = 640
        pending = b''
        resample_state = None
        preroll = deque(maxlen=15)
        votes = deque(maxlen=5)
        sentence = []
        started_at = 0.0
        quiet = voiced = 0
        recovering = False
        last_health = last_audio = time.monotonic()

        def reset_sentence():
            nonlocal sentence, quiet, voiced
            sentence = []
            quiet = voiced = 0
            preroll.clear()
            votes.clear()

        while not shutdown.is_set() and os.getppid() == parent_pid:
            now = time.monotonic()
            if now-last_health >= 0.5:
                emit('health', {})
                last_health = now
            try:
                kind, data, captured, wall, gap = audio.get(timeout=0.1)
            except queue.Empty:
                if not capture.is_alive() or now-last_audio > 4:
                    raise RuntimeError('Không còn nhận được mẫu từ microphone')
                continue
            last_audio = time.monotonic()
            if kind == 'error':
                raise RuntimeError(data)
            if gap or time.monotonic()-captured > 1:
                if not recovering:
                    emit('warning', 'Âm thanh bị mất mẫu; chờ yên rồi nói lại cả câu.')
                reset_sentence()
                recovering = True
                pending = b''
                resample_state = None
                continue
            if rate != 16000:
                data, resample_state = audioop.ratecv(data, 2, 1, rate, 16000, resample_state)
            pending += data
            while len(pending) >= frame_bytes:
                frame, pending = pending[:frame_bytes], pending[frame_bytes:]
                speech = vad.is_speech(frame, 16000)
                if recovering:
                    quiet = 0 if speech else quiet+1
                    if quiet >= 40:
                        recovering = False
                        reset_sentence()
                        emit('state', 'Đang nghe tiếng Việt — hãy nói cả câu')
                    continue
                if not sentence:
                    preroll.append(frame)
                    votes.append(speech)
                    if sum(votes) < 3:
                        continue
                    sentence = list(preroll)
                    # Timestamp đầu câu để không chạy lại lệnh cũ sau khi bấm dừng.
                    started_at = wall-(len(pending)/2/16000)-len(sentence)*0.02
                    voiced = sum(votes)
                    emit('state', 'Đang thu câu nói…')
                    continue
                sentence.append(frame)
                voiced += int(speech)
                quiet = 0 if speech else quiet+1
                if len(sentence) >= 400:
                    # Không lấy câu bị cắt giữa chừng làm yêu cầu điều hướng.
                    emit('rejected', 'Câu quá dài; hãy nói một yêu cầu ngắn rồi nghỉ.')
                    reset_sentence()
                    recovering = True
                    continue
                if quiet < 30:
                    continue
                if voiced < 6:
                    reset_sentence()
                    continue
                # Câu hoàn chỉnh được chuyển sang luồng giải mã, không chặn VAD.
                observed_at = wall-len(pending)/2/16000
                waveform = np.frombuffer(b''.join(sentence), dtype=np.int16).astype(np.float32)/32768.0
                try:
                    jobs.put_nowait((waveform, started_at, observed_at))
                except queue.Full:
                    # Không âm thầm bỏ lệnh khi máy quá tải. Node điều hướng
                    # nhận audio_error sẽ yêu cầu hủy tuyến đang thực hiện.
                    emit('warning', 'Nhận dạng quá tải; yêu cầu dừng tuyến. Chờ rồi nói lại.')
                reset_sentence()
    except Exception as error:
        emit('fatal', str(error))
    finally:
        shutdown.set()
        if capture is not None:
            capture.join(timeout=1)
        if decoder is not None:
            decoder.join(timeout=1)
