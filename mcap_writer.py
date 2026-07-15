from mcap.writer import Writer
import json


class McapWriter:
    def __init__(self, path):
        self.file = open(path, "wb")
        self.writer: Writer = Writer(self.file)
        self.writer.start()
        self.channels = {}
        
    def register(self, topic, fields):
        schema_id = self.writer.register_schema(
            name=topic.strip("/"),
            encoding="jsonschema",
            # 
            data=json.dumps({
                "type": "object", # 'object' = dict
                "properties": {k: {"type": v} for k, v in fields.items()} # dict{str,dict} properties is k-type:v-'number'
            }).encode()
        )
        self.channels[topic] = self.writer.register_channel(
            topic=topic,
            message_encoding="json",
            schema_id=schema_id
        )
    
    def write(self, topic, time, data):
        ns = int(time*1e9)
        self.writer.add_message(
            channel_id=self.channels[topic],
            log_time=ns,
            publish_time=ns,
            data=json.dumps(data).encode(),
        )
    def close(self):
        self.writer.finish()
        self.file.close()
    
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()