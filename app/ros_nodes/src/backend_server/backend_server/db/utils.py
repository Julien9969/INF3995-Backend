from databases import Database
from backend_server.db.session import SQLALCHEMY_DATABASE_URL


async def check_db_connected():
    try:
        database = Database(SQLALCHEMY_DATABASE_URL)

        print(database)
        if not database.is_connected:
            await database.connect()
            await database.execute("SELECT 1")
        print("Database is connected (^_^)")
    except Exception as e:
        print(
            "Looks like db is missing or is there is some problem in connection,see below traceback"
        )
        raise e


async def check_db_disconnected():
    try:
        database = Database(SQLALCHEMY_DATABASE_URL)
        if database.is_connected:
            await database.disconnect()
        print("Database is Disconnected (-_-) zZZ")
    except Exception as e:
        raise e
