import asyncio, fastapi
import pytest
from unittest.mock import  Mock, PropertyMock, patch, MagicMock
from backend_server.websocket.events import Events
from backend_server.websocket.event_handlers.map import MapManager, MapSubscriber, send_map_image, twos_comp_byte
from array import array


##                                              TEST CASES FOR listener callback

@patch.object(MapSubscriber, 'convertDataToBase64Str')
@patch("rclpy.node.Node.create_subscription")
def test_listener_callback(create_subscription_mock, mock_convertDataToBase64Str): #ASK MAJEED Permission to precise types
    create_subscription_mock.return_value = MagicMock()
    mock_convertDataToBase64Str.return_value = 'mock_base64_data'
    map_subscriber = MapSubscriber()

    occupancy_grid = Mock()
    info = Mock()
    info.width = 10
    info.height = 10
    occupancy_grid.info = info
    occupancy_grid.data = [1,2,3,4,5,6,7,8,9,10]
    
    
    map_subscriber.listener_callback(occupancy_grid)
    assert map_subscriber.newMapAvailable == True
    assert map_subscriber.base_64_map_img == 'data:image/bmp;base64,mock_base64_data'
    mock_convertDataToBase64Str.assert_called_once_with(occupancy_grid)
    
##                                              TEST CASES FOR grid_dimensions
    
@patch("rclpy.node.Node.create_subscription")
def test_get_grid_dimensions(create_subscription_mock):
    create_subscription_mock.return_value = MagicMock()
    map_subscriber = MapSubscriber()

    mock_grid = Mock()
    mock_grid.info.width = 10
    mock_grid.info.height = 20

    width, height = map_subscriber.get_grid_dimensions(mock_grid)

    assert width == 10
    assert height == 20

##                                              TEST CASES FOR calculate_msb_lsb

@patch("rclpy.node.Node.create_subscription")
@patch("backend_server.websocket.event_handlers.map.twos_comp_byte")
def test_calculate_msb_lsb(twos_comp_byte_mock, create_subscription_mock):
    twos_comp_byte_mock.side_effect = lambda x: x  # Mock twos_comp_byte to return its input

    create_subscription_mock.return_value = MagicMock()
    map_subscriber = MapSubscriber()
    msb, lsb = map_subscriber.calculate_msb_lsb(300)
    assert msb == 1
    assert lsb == 44
    twos_comp_byte_mock.assert_not_called()
    msb, lsb = map_subscriber.calculate_msb_lsb(500)
    assert msb == 1
    assert lsb == 244
    twos_comp_byte_mock.assert_called_once_with(244)

##                                              TEST CASES FOR create_data_array

@patch("rclpy.node.Node.create_subscription")
@patch("backend_server.websocket.event_handlers.map.twos_comp_byte")
@patch.object(MapSubscriber, 'append_grid_data_to_array')
def test_create_data_array(append_grid_data_to_array_mock, twos_comp_byte_mock, create_subscription_mock):

    create_subscription_mock.return_value = MagicMock()
    map_subscriber = MapSubscriber()
    twos_comp_byte_mock.side_effect = lambda x: 127 if x > 127 else x
    mock_grid = Mock()

    map_subscriber.create_data_array(10, 20, 0, 10, 0, 20, mock_grid)

    expected_data = array('b', [0x42, 0x4D, 127, 127, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 10, 0, 0x00, 0x00, 20, 0, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 127, 127, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    append_grid_data_to_array_mock.assert_called_once_with(expected_data, 10, 20, mock_grid)



##                                              TEST CASES FOR append_grid_data_to_array
    
@patch("rclpy.node.Node.create_subscription")
@patch("backend_server.websocket.event_handlers.map.MapSubscriber.append_point_value_to_data")
@patch("backend_server.websocket.event_handlers.map.MapSubscriber.add_padding_to_data")
def test_append_grid_data_to_array(add_padding_to_data_mock, append_point_value_to_data_mock, create_subscription_mock):
    create_subscription_mock.return_value = MagicMock()
    map_subscriber = MapSubscriber()
    mock_grid = Mock()
    mock_grid.data = [0, 1, 2, 3, 4, 5]

    data = array('b', [0, 0, 0])
    map_subscriber.append_grid_data_to_array(data, 2, 3, mock_grid)

    append_point_value_to_data_mock.assert_any_call(data, 0)
    append_point_value_to_data_mock.assert_any_call(data, 1)
    append_point_value_to_data_mock.assert_any_call(data, 2)
    append_point_value_to_data_mock.assert_any_call(data, 3)
    append_point_value_to_data_mock.assert_any_call(data, 4)
    append_point_value_to_data_mock.assert_any_call(data, 5)

    add_padding_to_data_mock.assert_called()

##                                              TEST CASES FOR append_point_value_to_data

# @patch("rclpy.node.Node.create_subscription")
# @patch("backend_server.websocket.event_handlers.map.twos_comp_byte")
# def test_append_point_value_to_data(twos_comp_byte_mock, create_subscription_mock):
#     create_subscription_mock.return_value = MagicMock()
#     map_subscriber = MapSubscriber()
#     twos_comp_byte_mock.side_effect = lambda x: 127 if x > 127 else x  # Mock twos_comp_byte to return a valid signed char value

#     data = array('b', [])
#     map_subscriber.append_point_value_to_data(data, -1)

#     expected_data = array('b', [127, 127, 127])
#     assert data == expected_data

#     twos_comp_byte_mock.assert_called_with(175)

#     data = array('b', [])
#     map_subscriber.append_point_value_to_data(data, 50)

#     expected_data = array('b', [127, 127, 127])
#     assert data == expected_data

#     twos_comp_byte_mock.assert_called_with(127)

##                                              TEST CASES FOR add_padding_to_data

@patch("rclpy.node.Node.create_subscription")
def test_add_padding_to_data(create_subscription_mock):
        
    create_subscription_mock.return_value = MagicMock()
    map_subscriber = MapSubscriber()
    data = array('b', [0, 0, 0])
    map_subscriber.add_padding_to_data(data, 1)
    expected_data = array('b', [0, 0, 0, 0])
    assert data == expected_data

    data = array('b', [0, 0, 0])
    map_subscriber.add_padding_to_data(data, 2)
    expected_data_2 = array('b', [0, 0, 0, 0, 0])
    assert data == expected_data_2


##                                              TEST CASE FOR stop_map_listener

def test_stop_map_listener():
    MapManager.mission_ongoing = True
    MapManager.stop_map_listener()
    assert not MapManager.mission_ongoing

##                                              TEST CASES FOR twos_comp_byte

def test_twos_comp_byte():
    assert twos_comp_byte(128) == -128
    assert twos_comp_byte(255) == -1
    assert twos_comp_byte(1) == 1
    assert twos_comp_byte(0) == 0
    
##                                              TEST CASES FOR send_map_image

@pytest.mark.asyncio
@patch("backend_server.websocket.event_handlers.map.sio.emit")
async def test_send_map_image(mock_emit):
    map_data = "test_map_data"
    await send_map_image(map_data)
    mock_emit.assert_called_once_with(Events.MAP_DATA.value, map_data)

    