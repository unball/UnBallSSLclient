
The packet defined in `messages_robocup_ssl_detection.proto` in ssl-vision/src/shared/proto contains the detection results of one camera's frame, including robots and balls

The packet defined in `messages_robocup_ssl_geometry.proto` contains geometry information, namely field dimensions and camera calibration information. This data is published by default at an interval of 3 seconds. The publish interval can be changed by editing the Interval (seconds) field under Publish Geometry -> Auto Publish. It can also be manually published by clicking the Publish! button in the Publish Geometry branch.

The files are already compiled here, to recompile the `.proto` files inside the `proto` folder as Python, you can use the `protoc` compiler with the `--python_out` option.

Compile the `.proto` files:

```sh
cd proto
protoc --python_out=. *.proto
```

This will generate Python files for each `.proto` file in the directory.

If you encounter the error `ModuleNotFoundError: No module named 'messages_robocup_ssl_detection_pb2'` inside of `messages_robocup_ssl_wrapper_pb2.py` this is due to an error with protoc python, change
```
import messages_robocup_ssl_detection_pb2 as messages__robocup__ssl__detection__pb2
import messages_robocup_ssl_geometry_pb2 as messages__robocup__ssl__geometry__pb2
```
to
```python
from . import messages_robocup_ssl_detection_pb2 as messages__robocup__ssl__detection__pb2
from . import messages_robocup_ssl_geometry_pb2 as messages__robocup__ssl__geometry__pb2
```

Inspired on the following projects, thanks for their valuable work:

- [python-ssl-client](https://github.com/Juniorlimaivd/python-ssl-client)
- [pySSLVision](https://github.com/project-neon/pySSLVision)