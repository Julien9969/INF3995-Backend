PI3 application backend
## Setup with docker compose
Access the api docs:
- http://localhost:8000/docs
- http://localhost:8000/redoc

### Test connection
**Show avaible topic**
```sh
ros2 topic list
```
**Move the robots**
```sh
ros2 topic pub --rate 1 /limo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
or request 
```sh
localhost:8000/api/ping
```

**Si ne fonctionne pas essayer, pour "refresh" ?**
```sh
ros2 topic info </topic> -v
```
**see the topic messages**
```sh
ros2 topic echo /limo/cmd_vel "or /limo/odom"...
```


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
cd /src/app/ros_nodes/src/backend_server
pytest --ignore=./test --ignore=./tests/mock --cov-config=.coveragerc --cov-report term-missing --cov=backend_server backend_server/
```

##### Test execution with one command (--ignore=./test is for ros2 lint exclude)
```sh
docker exec -it fastapi-container bash -c "cd /src/app/ros_nodes/src/backend_server && pytest --ignore=./test --cov-report term-missing --cov=backend_server backend_server/"
``` 

### Delete the docker and cache
At `docker-compose.yml` location run 
```sh
docker compose stop
docker compose rm
```
Prompt `Y` for the container to delete
```sh
docker rmi inf3995-backend-fastapi
docker rmi postgres:13
docker builder prune
docker images
```

Reset docker configuration (change db user)
```sh
docker compose down -v
```

## Unit Test tutorial
#### Tests are build to be ros2 independent so the installation of ros python libs is not required.

But pytest need to know the methods, classes etc of the ros2 lib.  
The start of the ros2 lib mock is in [app\ros_nodes\src\backend_server\backend_server\tests\mock](app\ros_nodes\src\backend_server\backend_server\tests\mock).

#### All test function should start with `test` to make pytest found them

If you need to create a new folder in **tests/mock** you will need to patch the import to make pytest use the mock as it's already done around line 25 in [conftest.py](app\ros_nodes\src\backend_server\backend_server\tests\conftest.py)

`-s` in pytest call to show the print in test



## Resources

Gitlab ci ignore test folder (ros2 dep)
pytest --ignore=./test --cov-config=.coveragerc --ignore=./tests/mock --cov-report term-missing --cov=backend_server backend_server/

[Fast API documentation](https://fastapi.tiangolo.com/) \
[Fastapi sql db tutorial](https://fastapi.tiangolo.com/tutorial/sql-databases/) \
[SQLAlchemy](https://www.sqlalchemy.org/) \
[Pydantic](https://pydantic-docs.helpmanual.io/)


## For Windows:

```bash
docker run --rm -p 5430:5432 -v postgres_data:/var/lib/postgresql/data -e POSTGRES_DB=inf3995 -e POSTGRES_USER=eq102 -e POSTGRES_PASSWORD=root -it postgres:13

docker run --rm -p 8000:8000 -v C://full-path/INF3995-Backend/app:/src/app -v /app/ros_nodes/build -v /app/ros_nodes/install -v /app/ros_nodes/log -e ROS_DOMAIN_ID=62 -e TERM='xterm-256color' -e SQLALCHEMY_DATABASE_HOST=host.docker.internal -e WATCHFILES_FORCE_POLLING=true -it fastapi bash -c "/bin/bash -c '/start-app.sh'"
```