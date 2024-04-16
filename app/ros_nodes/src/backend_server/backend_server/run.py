import logging
import uvicorn
import sys, os

sys.path.append(".")

from backend_server.classes.constants import PORT


def main():
    logging.debug("Starting backend server")
    if os.getenv("DEPLOYMENT", "DEV") == "DEV":
        uvicorn.run("src.backend_server.backend_server.app:app", host="0.0.0.0", port=PORT, workers=4, reload=True)
    else:
        uvicorn.run("src.backend_server.backend_server.app:app", host="0.0.0.0", port=PORT, workers=4, reload=False)

if __name__ == "__main__":
    main()
