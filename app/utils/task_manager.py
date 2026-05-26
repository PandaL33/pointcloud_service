# app/utils/task_manager.py
# Author: hzc
# Created: 2026-05-26
# Description: 基于 Redis 的任务状态管理器
import redis
import json
import logging
from typing import Optional, Dict, Any
from app.config import settings

logger = logging.getLogger(__name__)

class RedisTaskManager:
    """基于 Redis 的任务状态管理器"""
    
    def __init__(
        self,
        host: str = None,
        port: int = None,
        db: int = None,
        password: str = None,
        task_ttl: int = None
    ):
        """
        初始化 Redis 任务管理器
        
        Args:
            host: Redis 服务器地址
            port: Redis 端口
            db: Redis 数据库编号
            password: Redis 密码
            task_ttl: 任务状态过期时间（秒）
        """
        self.host = host or settings.redis_host
        self.port = port or settings.redis_port
        self.db = db or settings.redis_db
        self.password = password or settings.redis_password
        self.task_ttl = task_ttl or settings.redis_task_ttl
        self.key_prefix = "task:status:"
        
        try:
            self.redis_client = redis.Redis(
                host=self.host,
                port=self.port,
                db=self.db,
                password=self.password,
                decode_responses=True,  # 自动解码字符串
                socket_connect_timeout=5,
                socket_timeout=5
            )
            logging.info(f"Redis 连接参数: {self.host}:{self.port}/{self.password}")
            # 测试连接
            self.redis_client.ping()
            logger.info(f"Redis 连接成功: {self.host}:{self.port}/{self.db}")
        except redis.ConnectionError as e:
            logger.error(f"Redis 连接失败: {e}")
            raise
        except Exception as e:
            logger.error(f"Redis 初始化失败: {e}")
            raise
    
    def _get_key(self, task_id: str) -> str:
        """获取任务的 Redis key"""
        return f"{self.key_prefix}{task_id}"
    
    def create_task(self, task_id: str, initial_status: Dict[str, Any]) -> bool:
        """
        创建任务状态
        
        Args:
            task_id: 任务 ID
            initial_status: 初始状态字典
            
        Returns:
            是否创建成功
        """
        try:
            key = self._get_key(task_id)
            # 将状态字典序列化为 JSON 字符串
            status_json = json.dumps(initial_status, ensure_ascii=False)
            # 设置值并添加过期时间
            self.redis_client.setex(key, self.task_ttl, status_json)
            logger.debug(f"创建任务状态: {task_id}")
            return True
        except Exception as e:
            logger.error(f"创建任务状态失败 {task_id}: {e}")
            return False
    
    def get_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        """
        获取任务状态
        
        Args:
            task_id: 任务 ID
            
        Returns:
            任务状态字典，如果不存在则返回 None
        """
        try:
            key = self._get_key(task_id)
            status_json = self.redis_client.get(key)
            if status_json is None:
                return None
            # 反序列化 JSON 字符串
            return json.loads(status_json)
        except Exception as e:
            logger.error(f"获取任务状态失败 {task_id}: {e}")
            return None
    
    def update_task(self, task_id: str, status: Dict[str, Any]) -> bool:
        """
        更新任务状态
        
        Args:
            task_id: 任务 ID
            status: 新的状态字典
            
        Returns:
            是否更新成功
        """
        try:
            key = self._get_key(task_id)
            # 先检查任务是否存在
            if not self.redis_client.exists(key):
                logger.warning(f"任务不存在，无法更新: {task_id}")
                return False
            
            # 将状态字典序列化为 JSON 字符串
            status_json = json.dumps(status, ensure_ascii=False)
            # 更新值并刷新过期时间
            self.redis_client.setex(key, self.task_ttl, status_json)
            logger.debug(f"更新任务状态: {task_id}")
            return True
        except Exception as e:
            logger.error(f"更新任务状态失败 {task_id}: {e}")
            return False
    
    def delete_task(self, task_id: str) -> bool:
        """
        删除任务状态
        
        Args:
            task_id: 任务 ID
            
        Returns:
            是否删除成功
        """
        try:
            key = self._get_key(task_id)
            result = self.redis_client.delete(key)
            logger.debug(f"删除任务状态: {task_id}")
            return result > 0
        except Exception as e:
            logger.error(f"删除任务状态失败 {task_id}: {e}")
            return False
    
    def task_exists(self, task_id: str) -> bool:
        """
        检查任务是否存在
        
        Args:
            task_id: 任务 ID
            
        Returns:
            任务是否存在
        """
        try:
            key = self._get_key(task_id)
            return self.redis_client.exists(key) > 0
        except Exception as e:
            logger.error(f"检查任务存在性失败 {task_id}: {e}")
            return False
    
    def get_all_tasks(self) -> Dict[str, Dict[str, Any]]:
        """
        获取所有任务状态（慎用，可能影响性能）
        
        Returns:
            所有任务状态字典 {task_id: status}
        """
        try:
            pattern = f"{self.key_prefix}*"
            tasks = {}
            for key in self.redis_client.scan_iter(match=pattern):
                task_id = key.replace(self.key_prefix, "")
                status = self.get_task(task_id)
                if status:
                    tasks[task_id] = status
            return tasks
        except Exception as e:
            logger.error(f"获取所有任务状态失败: {e}")
            return {}
    
    def cleanup_expired_tasks(self) -> int:
        """
        清理过期任务（Redis 会自动处理，此方法主要用于统计）
        
        Returns:
            清理的任务数量（实际上 Redis 已自动清理）
        """
        # Redis 的 TTL 机制会自动清理过期键
        # 这个方法可以用于监控和统计
        return 0


# 创建全局任务管理器实例（延迟初始化）
_task_manager = None

def get_task_manager() -> RedisTaskManager:
    """
    获取全局任务管理器实例（单例模式）
    
    Returns:
        RedisTaskManager 实例
    """
    global _task_manager
    if _task_manager is None:
        try:
            _task_manager = RedisTaskManager()
        except Exception as e:
            logger.error(f"任务管理器初始化失败: {e}")
            raise
    return _task_manager
