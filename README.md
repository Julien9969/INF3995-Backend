PI3 application backend
## Setup

### Python

Create a virtual environment :
```bash
python3 -m venv .venv
```
or in vscode : \
Ctrl+Shift+P -> Python: Select Interpreter -> Enter interpreter path -> Enter path to venv/bin/python3

Activate the virtual environment (Linux):
```bash
source ./.venv/bin/activate
```

Activate the virtual environment (Windows):
```
./.venv/Scripts/activate
```

Install dependencies (in the virtual environment):
```bash
pip install -r requirements.txt
```

## Usage

Start the server (at the root of the project):
```
uvicorn app.main:app --reload
```


Open the docs in a browser :
http://localhost:8000/docs

## setup with docker

make sure that docker is installed on your computer by running
```
docker -v
```

Go to the path where the `Dockerfile` is then run
```
docker build -t projet3 .
```

Check if the image has been created by run
```
docker images
```
docker build -t projet3 .
docker run -p 3200:8000 -v $(pwd):/app --name pj3 -it projet3



docker logs -f pj3

docker builder prune
docker exec -it pj3 psql -U eq102 -d inf3995 


postgres UP ?
```
docker exec -it pj3 pg_isready
```

delete and clean the docker install
```
docker container stop pj3
docker rm pj3
docker rmi projet3
docker builder prune
docker images
```



## Resources

[Fast API documentation](https://fastapi.tiangolo.com/) \
[Fastapi sql db tutorial](https://fastapi.tiangolo.com/tutorial/sql-databases/) \
[SQLAlchemy](https://www.sqlalchemy.org/) \
[Pydantic](https://pydantic-docs.helpmanual.io/)