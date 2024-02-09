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
```sh
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

## Setup with docker compose
Access the api docs:
- http://localhost:8000/docs
- http://localhost:8000/redoc

### Install
make sure that **docker** and **docker compose / docker-compose** is installed on your computer by running

```sh
docker -v
docker compose -v
```
or 
```sh
docker -v
docker-compose
```
For ubuntu it seems to be `docker compose` instead of `docker-compose`

</br>

At `docker-compose.yml` location run 
```sh
docker compose up
```

Check if the image has been created
```sh
docker images
```
You should see a repository named `inf3995-backend-fastapi`

To see detail status of the container created (fastapi and postgres db)
```sh
docker ps -a
```
`inf3995-backend-fastapi` and `postgres:13` should be in IMAGE column


### Use the created images (pas sur du nom)

#### **Start the server**
```sh
docker compose start
```

**Open live logs of the server**
```sh
docker compose logs -f
```

**Open a bash of fast-api**
```sh
docker exec -it fastapi-container bash
```

**Open a bash of the database**
```sh
docker exec -it inf3995-backend-db-1 bash
```
**Open postgres command prompt**
```sh
docker exec -it inf3995-backend-db-1 psql -U eq102 -d inf3995 
``````
**list all db tables**
```sql
\dt
```
**Display elements of something table**
```sql
select * from something;
```
#### Stop the server
```sh
docker compose stop
```

### Run the tests
Open a bash of fast-api
```sh
docker exec -it fastapi-container bash
```
Run the tests + coverage
```sh
cd ros_nodes/src/backend_server/  
pytest --cov-report term-missing --cov=backend_server backend_server/
```

##### Test execution with one command
```sh
docker exec -it fastapi-container bash -c "cd ros_nodes/src/backend_server/ && pytest --cov-report term-missing --cov=backend_server backend_server/"
``` 

### Delete the docker and cache
At `docker-compose.yml` location run 
```sh
docker compose stop
docker compose rm
```
Prompt `Y` for the container to delete
```sh
docker rmi fastapi-container
docker rmi postgres:13
docker builder prune
docker images
```

Reset docker configuration (change db user)
```sh
docker compose down -v
```

in progress  
uvicorn backend_server.main:app --reload --host 0.0.0.0 --port 8000


## Resources

[Fast API documentation](https://fastapi.tiangolo.com/) \
[Fastapi sql db tutorial](https://fastapi.tiangolo.com/tutorial/sql-databases/) \
[SQLAlchemy](https://www.sqlalchemy.org/) \
[Pydantic](https://pydantic-docs.helpmanual.io/)