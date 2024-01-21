FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive

# Install Python and pip
RUN apt-get update && apt-get install -y python3 python3-pip

# Set the working directory
WORKDIR /app

# Copy the requirements file into the container
COPY requirements.txt /app/

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# Install PostgreSQL
# Uncomment and modify the following lines if you want to install and configure PostgreSQL
RUN apt-get install -y postgresql postgresql-contrib postgresql-server-dev-all

# Switch to the postgres user and create a new database and user
USER postgres

# Run commands to initialize the database cluster, create a new database and user
RUN /etc/init.d/postgresql start && \
    psql --command "CREATE USER eq102 WITH SUPERUSER PASSWORD 'root';" && \
    createdb -O eq102 inf3995

RUN service postgresql start && \
    sleep 5 && \
    pg_isready -h localhost -p 5432

EXPOSE 8000

# Run uvicorn when the container launches
CMD ["/bin/bash"]
# CMD ["uvicorn", "app.main:app", "--reload"]


# # Set environment variables for PostgreSQL
# ENV POSTGRES_DB inf3995
# ENV POSTGRES_USER eq102
# ENV POSTGRES_PASSWORD root

# Expose port 3200 to the world outside this container







# # Stage 1: Build Python dependencies
# FROM python:3.11 as builder
# 
# WORKDIR /app
# 
# COPY requirements.txt /app/
# 
# RUN pip install --no-cache-dir -r requirements.txt
# 
# # Stage 2: Build the final image
# FROM library/postgres
# 
# WORKDIR /app
# 
# # Copy Python dependencies from the builder stage
# COPY --from=builder /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages
# 
# # Set environment variables for PostgreSQL
# ENV POSTGRES_DB inf3995
# ENV POSTGRES_USER eq102
# ENV POSTGRES_PASSWORD root

# Uncomment the following lines if you want to install PostgreSQL
# RUN apt-get update && apt-get install -y postgresql postgresql-contrib postgresql-server-dev-all

# Uncomment the following lines if you want to create a PostgreSQL database and user
# USER postgres
# RUN /etc/init.d/postgresql start && \
#     psql --command "CREATE USER eq102 WITH SUPERUSER PASSWORD 'root';" && \
#     createdb -O eq102 inf3995

# Expose port 3200 to the world outside this container
# EXPOSE 3200
# 
# # Run uvicorn when the container launches
# CMD ["uvicorn", "app.main:app", "--reload"]
# 


# # Use an official Python runtime as a parent image
# FROM python:3.11
# 
# # Set the working directory to /app
# WORKDIR /app
# 
# # Copy the requirements file into the container at /app
# COPY requirements.txt /app/
# 
# # Install any needed packages specified in requirements.txt
# RUN pip install --no-cache-dir -r requirements.txt
# 
# # Install PostgreSQL
# # RUN apt-get update && apt-get install -y postgresql postgresql-contrib postgresql-server-dev-all
# 
# FROM library/postgres
# # Set environment variables
# ENV POSTGRES_DB inf3995
# ENV POSTGRES_USER eq102
# ENV POSTGRES_PASSWORD root
# 
# # Create PostgreSQL database and user
# # USER postgres
# # RUN /etc/init.d/postgresql start && \
# #     psql --command "CREATE USER eq102 WITH SUPERUSER PASSWORD 'root';" && \
# #     createdb -O eq102 inf3995
# 
# # Expose port 8000 to the world outside this container
# EXPOSE 3200
# 
# # Run uvicorn when the container launches
# CMD ["uvicorn", "app.main:app", "--reload"]
# # "--host", "0.0.0.0", "--port", "80"]