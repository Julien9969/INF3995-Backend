import logging
import uvicorn
import sys

sys.path.append(".")  # TODO: could be changed, messes with IDE

from backend_server.classes.constants import PORT


def main():
    logging.debug("Starting backend server")
    uvicorn.run("src.backend_server.backend_server.app:app", host="0.0.0.0", port=PORT, workers=4, reload=True)


if __name__ == "__main__":
    main()
