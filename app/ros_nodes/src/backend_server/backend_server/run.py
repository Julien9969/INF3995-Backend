import uvicorn
import sys
import os
from dotenv import load_dotenv

sys.path.append(".")
load_dotenv()


def main():
    print("Starting server", os.listdir("."))
    uvicorn.run("src.backend_server.backend_server.app:app", host="0.0.0.0", port=8000,workers=4, reload=True)


if __name__ == "__main__":
    main()
