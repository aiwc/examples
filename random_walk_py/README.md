# random_walk_py (English)
Random Walk example for the AI World Cup (Python)

A python implementation for the random walk example described in: [here](https://github.com/aiwc/examples/tree/master/random_walk)

## Contact

***Questions, issues, pull requests:*** @lfelipesv (kakaoid: lfelipe)

## Requirements

Webots >= 8.6.2, Python 3

Obs: For versions <= 8.6.1, Webots will require Python 2.7.

Other requirements: [autobahn, twisted](http://autobahn.readthedocs.io/en/latest/installation.html), u-msgpack-python

```shell
# Install autobahn[twisted] using PIP
pip install autobahn[twisted]

# Install u-msgpack-python using PIP
pip install u-msgpack-python
```

If you do not have pip installed, install using this [link](https://pip.pypa.io/en/stable/installing/).

## How to run this code using Windows?

You will need [Pyinstaller](http://www.pyinstaller.org/) to create an executable (.exe file).

```shell
# Install Pyinstaller using PIP
pip install pyinstaller

# In the random_walk_py directory run
pyinstaller random_walk_py.py
```

Use the executable generated in the directory /dist/random_walk_py as your TeamExecutable file using the *Webots(Windows)* API.

## How to run this code using Linux?

Change the first line of the python code adding the path to your python environment. To find your python path run:

```shell
which python
```

The new line will be: #!usr/bin/python3

Use the path to the python code /examples/random_walk_py/random_walk_py.py as your TeamExecutable file using the *Webots(Linux)* API.

Give permission to execute random_walk_py file with the following command:

```shell
chmod +x random_walk_py.py
```

# random_walk_py (Korean)

AI World Cup 을 위한 파이썬(Python) 랜덤 워크 예시 코드입니다. 

['이곳'](https://github.com/aiwc/examples/tree/master/random_walk)에서 파이썬으로 구현된 랜덤워크 코드를 볼 수 있습니다.

## Contact

***질문, 문제 사항, 요청 사항이 있으시면 필립에게 문의하세요:*** @lfelipesv (카카오톡 아이디: lfelipe)

## Requirements

8.6.1 버전 이하의 Webots 에서는 파이썬 2.7. 버전이 사용되어야 합니다. 

그 외에 필요한 것들: [autobahn, twisted](http://autobahn.readthedocs.io/en/latest/installation.html)

```shell
# Install autobahn[twisted] using PIP
pip install autobahn[twisted]
```

pip가 설치되어있지 않다면 ['이곳'](https://pip.pypa.io/en/stable/installing/)에서 다운받을 수 있습니다.

## 윈도우에서 코드를 실행하는 방법

executable(.exe 파일)을 생성하기 위해서는 [Pyinstaller](http://www.pyinstaller.org/) 가 필요합니다.

```shell
# Install Pyinstaller using PIP
pip install pyinstaller

# In the random_walk_py directory run
pyinstaller random_walk_py.py
```

Team Executable 파일을 만들 때 Webots(Windows) API의 디렉토리dist/random_walk_py 에 생성되어있는 executable(.exe 파일)을 이용하세요. 
