import json
from fastapi import APIRouter, status, responses
from backend_server.schemas.schemas import File, FileId
from pydantic import BaseModel
from .files_base import ROSFilesBase

import asyncio, os

router = APIRouter(include_in_schema=True)

@router.get("/tree/{robot_id}")
async def get_files_tree(robot_id: int) -> responses.JSONResponse:

    message, data = await ROSFilesBase.get_files_tree(robot_id)

    if message == "Error":
        return responses.JSONResponse (
            message if not message is None else "Can't connect to files service !",
            status_code=status.HTTP_404_NOT_FOUND if not message is None else status.HTTP_503_SERVICE_UNAVAILABLE
        )
    else:
        return responses.JSONResponse (
            data,
            status_code=status.HTTP_200_OK
        )

@router.get("/file")
async def get_file(robot_id: int, name: str, id: int) -> responses.JSONResponse:

    message, content = await ROSFilesBase.get_file(robot_id, name, id)

    if message == "Error":
        return responses.JSONResponse (
            content,
            status_code=status.HTTP_404_NOT_FOUND
        )

    file_json = {"name": content.name, "id": content.id, "content": content.content}

    return responses.JSONResponse(
        content=file_json,
        status_code=status.HTTP_200_OK
    )

@router.post("/save/{robot_id}")
async def post_file(robot_id: int, file: File) -> responses.JSONResponse:

    message, content = await ROSFilesBase.edit_file(robot_id, file)

    if message == "Error":
        return responses.JSONResponse (
            content,
            status_code=status.HTTP_404_NOT_FOUND
        )
    return responses.JSONResponse (
        "File saved",
        status_code=status.HTTP_200_OK
    )

@router.patch("/update/{robot_id}")
async def post_file(robot_id: int) -> responses.JSONResponse:

    message, content = await ROSFilesBase.update_robot(robot_id)

    if message == "Error":
        return responses.JSONResponse (
            content,
            status_code=status.HTTP_404_NOT_FOUND
        )
    
    return responses.JSONResponse (
        content,
        status_code=status.HTTP_200_OK
    )

@router.get("/test")
async def test() -> responses.JSONResponse:

    print("test")
    return responses.JSONResponse (
        { 'data': "test" },
        status_code=status.HTTP_200_OK
    )
