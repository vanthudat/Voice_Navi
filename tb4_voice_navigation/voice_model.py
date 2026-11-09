"""Chuẩn bị model offline; PhoWhisper được đổi sang CTranslate2 int8 một lần."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

PHO_REPOSITORY = 'vinai/PhoWhisper-small'
PHO_REVISION = 'a86b604c346caf7148c37512eafe783a16420adb'


def cache_directory():
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))
    return cache/'tb4_voice_navigation'


def default_voice_model_path():
    return cache_directory()/'phowhisper-small'


def complete_model(path):
    return all((path/name).is_file() and (path/name).stat().st_size > 0
               for name in ('model.bin', 'config.json', 'tokenizer.json'))


def prepare_phowhisper(destination):
    from huggingface_hub import snapshot_download
    # Chỉ tải trọng số và cấu hình từ model chính thức, cố định revision.
    source = cache_directory()/'phowhisper-small-source'
    snapshot_download(repo_id=PHO_REPOSITORY, revision=PHO_REVISION,
        local_dir=str(source), allow_patterns=['*.json', '*.txt', 'pytorch_model.bin'])
    # Tách thư viện dùng khi chuyển model, tránh đổi phiên bản thư viện của ROS.
    environment = cache_directory()/'phowhisper-converter'
    python = environment/'bin/python'
    marker = environment/'dependencies-v3.ready'
    if not python.is_file():
        subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages',
                        str(environment)], check=True)
    if not marker.is_file():
        subprocess.run([str(python), '-m', 'pip', 'install',
            'transformers==4.57.6', 'huggingface-hub>=0.23.2,<1',
            'tokenizers>=0.22,<=0.23', 'safetensors>=0.4.1,<1', 'Pillow>=10,<12'], check=True)
        marker.touch()
    # Chưa hoàn tất thì không cho speech_node thấy model đích.
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        converted = Path(temp)/'model'
        print('Đang chuyển PhoWhisper-small sang CTranslate2 int8…', flush=True)
        subprocess.run([str(python), '-c', 'from ctranslate2.converters.transformers import main; main()',
            '--model', str(source), '--output_dir', str(converted),
            '--quantization', 'int8', '--copy_files', 'tokenizer.json',
            'preprocessor_config.json'], check=True)
        if not complete_model(converted):
            raise RuntimeError('Chuyển model chưa tạo đủ file cần thiết')
        converted.rename(destination)


def main():
    parser = argparse.ArgumentParser(description='Chuẩn bị model nhận giọng nói offline')
    parser.add_argument('--model', choices=['phowhisper-small', 'whisper-small', 'whisper-base'],
                        default='phowhisper-small')
    # Giữ tùy chọn cũ để vẫn tải được Whisper khi cần đối chiếu.
    parser.add_argument('--size', choices=['base', 'small'], default=None,
                        help='Tải Whisper gốc theo kích thước; ưu tiên hơn --model')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    name = 'whisper-'+args.size if args.size else args.model
    destination = (args.output or cache_directory()/name).expanduser().resolve()
    if complete_model(destination):
        print('Model đã sẵn sàng: '+str(destination))
        return
    if destination.exists():
        raise FileExistsError('Thư mục đích chưa đủ model; chọn --output khác: '+str(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    print('Chuẩn bị '+name+' tại '+str(destination), flush=True)
    if name == 'phowhisper-small':
        prepare_phowhisper(destination)
    else:
        from huggingface_hub import snapshot_download
        snapshot_download(repo_id='Systran/faster-'+name, local_dir=str(destination),
            allow_patterns=['model.bin', 'config.json', 'tokenizer.json',
                            'preprocessor_config.json', 'vocabulary.*'])
    if not complete_model(destination):
        raise FileNotFoundError('Model chưa đủ file: '+str(destination))
    print('Model đã sẵn sàng: '+str(destination), flush=True)


if __name__ == '__main__':
    main()
