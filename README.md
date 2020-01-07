# Rocket_Attitude_Control

ラズパイでロケットの姿勢制御をするためのコード

# Features

[pipenv](https://github.com/pypa/pipenv) で環境を作成します。pipenv の使い方は[公式ドキュメント](https://pipenv-ja.readthedocs.io/ja/translate-ja/)を参照してください。

```shell
pip install pipenv
pipenv install --python 3.7.2
```

# Requirement

* Python 3.7.2
* pigpio 1.44
* libi2c-dev
* smbus

# Installation

Install package with pipenv command.

```bash
pipenv install pigpio~=1.44
pipenv install smbus
pipenv install libi2c-dev
```

# Usage

Run "attitude_control.py"

```bash
git clone https://github.com/kure-kosen-water-rocket/rocket_attitude_control
pipenv shell
sudo pigpiod
python attitude_control.py
```

# Note

ラズベリーパイ上でのみ動作確認済みです
