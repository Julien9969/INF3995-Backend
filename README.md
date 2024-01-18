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

## Resources

[Fast API documentation](https://fastapi.tiangolo.com/) \
[Fastapi sql db tutorial](https://fastapi.tiangolo.com/tutorial/sql-databases/) \
[SQLAlchemy](https://www.sqlalchemy.org/) \
[Pydantic](https://pydantic-docs.helpmanual.io/)