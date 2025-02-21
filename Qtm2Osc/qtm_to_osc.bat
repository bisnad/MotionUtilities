set CONDA_PATH=C:\Users\%USERNAME%\anaconda3
set ENV_NAME=qtm
set PYTHON_VERSION=3.8

call %CONDA_PATH%\Scripts\activate.bat
call conda activate %ENV_NAME%
python qtm_to_osc.py
pause