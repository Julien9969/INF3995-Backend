import uvicorn, sys,os

sys.path.append(".")
def main():
    print("Starting server", os.listdir("."))
    uvicorn.run("src.backend_server.backend_server.app:app", workers=4, host="0.0.0.0", port=8000, reload=True)

if __name__ == "__main__":
    main()