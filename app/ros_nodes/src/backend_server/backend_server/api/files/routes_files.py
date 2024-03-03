from fastapi import APIRouter, status, responses
from pydantic import BaseModel

import asyncio, os

router = APIRouter(include_in_schema=True)

@router.get("/tree")
async def get_files_tree() -> responses.JSONResponse:

    data = None
    with open('./src/backend_server/backend_server/api/files/file_trees.json', 'r', encoding='utf-8') as file:
        data = file.read()

    return responses.JSONResponse (
        data,
        status_code=status.HTTP_200_OK
    )

@router.get("/file/{fileName}")
async def get_file(fileName: str) -> responses.JSONResponse:
    
    return responses.JSONResponse (
        { 'data': "file " + fileName }, 
        status_code=status.HTTP_200_OK
    )


class File(BaseModel):
    fileName: str
    fileContent: str

@router.post("/save")
async def post_file(file: File) -> responses.JSONResponse:

    print(file.fileName)
    print(file.fileContent[0:20])

    return responses.JSONResponse (
        { 'data': "file " + file.fileName + " created" }, 
        status_code=status.HTTP_200_OK
    )
