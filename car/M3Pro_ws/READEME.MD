# 快捷启动指令

## 1.启动底层主板代理

```
 sh start_agent.sh
```

## 6.slam建图

### 1.键盘控制节点




常见问题及解决方案：
1.ASR语音识别报错：
（1）ASR Error:access denied
原因：语音识别模型的免费额度用完、或用户欠费
解决：检查阿里云账号是否欠费、帐号是否处于可用状态、如果模型额度用完，请及时参考教程更换模型

ERROR:dashscope:Request failed, request_id: b4ff9d1f15704029bbd6ed593b0ccb2c, http_code: 44 error_name: Arrearage, error_message: Access denied, please make sure your account is in good standing.
ERROR:dashscope:{"status_code": 44, "request_id": "b4ff9d1f15704029bbd6ed593b0ccb2c", "code": "Arrearage", "message": "Access denied, please make sure your account is in good standing.", "output": null, "usage": null}
ASR Error:ASR Error:Access denied, please make sure your account is in good standing.
[ERROR] [1790088615.082098753] [rviz2]: Error retrieving file [package://M3Pro/meshes/rwheel2.STL]: Package [M3Pro] does not exist
[INFO] [1790088616.444122314] [rviz]: Message Filter dropping message: frame 'base_link' at time 1790088615.022 for reason 'discarding message because the queue is full'

