import pytest
from unittest.mock import Mock

# Direct import assuming 'shanaka' is a package in the project structure
from py_lib_digitaltwin.MQTTClientHandler import MQTTClientHandler

def test_on_connect():
    handler = MQTTClientHandler()
    mock_client = Mock()
    handler.on_connect(mock_client, None, None, 0)
    # No exception should be raised, and the output should indicate successful connection.

def test_on_disconnect():
    handler = MQTTClientHandler()
    mock_client = Mock()
    handler.on_disconnect(mock_client, None, 0)
    # No exception should be raised, and the output should indicate disconnection.

def test_on_publish():
    handler = MQTTClientHandler()
    mock_client = Mock()
    handler.on_publish(mock_client, None, 1)
    # No exception should be raised, and the output should indicate successful publishing.

def test_on_subscribe():
    handler = MQTTClientHandler()
    mock_client = Mock()
    handler.on_subscribe(mock_client, None, 1, [0])
    # No exception should be raised, and the output should indicate successful subscription.

def test_on_message_with_valid_xml():
    handler = MQTTClientHandler()
    mock_client = Mock()
    mock_message = Mock()
    mock_message.payload = b"<root><client_id>123</client_id><sensor_id>456</sensor_id><msg_id>789</msg_id><value>25.5</value></root>"
    mock_message.topic = "test/topic"
    handler.on_message(mock_client, None, mock_message)
    # No exception should be raised, and the output should indicate successful message processing.

def test_on_message_with_invalid_xml():
    handler = MQTTClientHandler()
    mock_client = Mock()
    mock_message = Mock()
    mock_message.payload = b"Invalid XML"
    mock_message.topic = "test/topic"
    handler.on_message(mock_client, None, mock_message)
    # No exception should be raised, and the output should indicate invalid XML payload.

def test_unsubscribe_from_topics():
    handler = MQTTClientHandler()
    mock_client = Mock()
    topics = ["topic1", "topic2"]
    handler.unsubscribe_from_topics(mock_client, topics)
    # Ensure unsubscribe is called for each topic.
    mock_client.unsubscribe.assert_any_call("topic1")
    mock_client.unsubscribe.assert_any_call("topic2")
