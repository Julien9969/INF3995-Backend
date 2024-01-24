FROM tiangolo/uvicorn-gunicorn-fastapi:python3.8
WORKDIR /src
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
# COPY ./app ./app
# CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "80"]
