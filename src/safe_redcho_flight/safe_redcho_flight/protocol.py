import hashlib
import json
import math


def fingerprint(cfg):
    return hashlib.sha256(json.dumps({k: v for k, v in cfg.items() if not k.startswith('_')},
                                     sort_keys=True, allow_nan=False).encode()).hexdigest()


def encode(value):
    return json.dumps(value, separators=(',', ':'), allow_nan=False)


def decode(text):
    if len(text) > 100000: raise ValueError('Control packet exceeds size bound')
    def reject(value): raise ValueError('Nonfinite JSON token '+value)
    value = json.loads(text, parse_constant=reject)
    if not isinstance(value, dict): raise ValueError('Control packet must be an object')
    return value
