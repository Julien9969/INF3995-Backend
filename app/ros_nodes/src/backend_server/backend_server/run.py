import logging
import sys

import uvicorn

sys.path.append(".")  # TODO: could be changed, messes with IDE


def main():
    logging.debug("Starting backend server")
    uvicorn.run("src.backend_server.backend_server.app:app", host="0.0.0.0", port=8000, workers=4, reload=True)


if __name__ == "__main__":
    main()
