from fastapi import status 


def test_ping(client, normal_user_token_headers):
    response = client.get("/api/ping/")
    assert response.status_code == 200
    assert str(response.json()["data"]).find("pong!") != -1
