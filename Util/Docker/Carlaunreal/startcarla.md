
## 使用方式

```bash
cd /home/qiyuan/UnrealEngine/CarlaUE5

# 启动两个实例
docker compose up -d

# 查看状态
docker compose ps

# 查看日志
docker compose logs -f carla1
docker compose logs -f carla2

# 停止全部
docker compose down
```

## 客户端连接

```python
import carla

client1 = carla.Client('localhost', 2000)  # 实例1
client2 = carla.Client('localhost', 2010)  # 实例2
```

两个实例分别监听 `2000` 和 `2010` 端口，互不干扰。