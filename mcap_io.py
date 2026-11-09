"""Portable MCAP JSON records; no visualization SDK or WebSocket server."""
import base64
import json
from mcap.writer import Writer

class McapSink:
    def __init__(self, path):
        self.file = open(path, 'xb')
        self.writer = Writer(self.file)
        self.writer.start(profile='panthera-json', library='panthera-viser')
        self.channels = {}
    def log(self, topic, payload, stamp, schema_name='panthera.Record', schema=None):
        if topic not in self.channels:
            sid = self.writer.register_schema(schema_name, 'jsonschema', json.dumps(schema or {'type':'object'}).encode())
            self.channels[topic] = self.writer.register_channel(topic, 'json', sid)
        self.writer.add_message(self.channels[topic], int(stamp), json.dumps(payload,allow_nan=False).encode(), int(stamp))
    def image(self, topic, data, stamp, format='jpeg', **metadata):
        self.log(topic, dict(format=format,data=base64.b64encode(data).decode(),**metadata), stamp, 'panthera.Image')
    def close(self):
        try:self.writer.finish()
        finally:self.file.close()

def decode_image(schema, channel, message):
    if channel.message_encoding=='json':
        return base64.b64decode(json.loads(message.data)['data'])
    # Read existing protobuf recordings without the former visualization SDK.
    from mcap_protobuf.decoder import DecoderFactory
    return bytes(DecoderFactory().decoder_for(channel.message_encoding,schema)(message.data).data)
