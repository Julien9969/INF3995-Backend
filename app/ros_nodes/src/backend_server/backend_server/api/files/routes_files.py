from fastapi import APIRouter, Query, status, responses
from backend_server.schemas.schemas import File, FileId
from pydantic import BaseModel
from .files_base import ROSFilesBase

import asyncio, os

router = APIRouter(include_in_schema=True)

@router.get("/tree/{robot_id}")
async def get_files_tree(robot_id: int) -> responses.JSONResponse:

    data = ROSFilesBase.get_files_tree(robot_id)

    return responses.JSONResponse (
        data,
        status_code=status.HTTP_200_OK
    )

@router.get("/file")
async def get_file(name: str, id: int) -> responses.JSONResponse:

    file_instance: File = ROSFilesBase.get_file()

    file_dict = {"name": file_instance.name, "id": file_instance.id, "content": file_instance.content}

    return responses.JSONResponse(
        content=file_dict,
        status_code=status.HTTP_200_OK
    )

@router.post("/save")
async def post_file(file: File) -> responses.JSONResponse:

    print(file.name)
    print(file.content[0:20])

    return responses.JSONResponse (
        { 'data': "file " + file.name + " created" }, 
        status_code=status.HTTP_200_OK
    )
