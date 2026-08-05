"""Remote storage interface for cloud and remote storage solutions."""

import logging
from typing import Any, Dict, Optional

from .backends import LocalBackend, S3Backend, SFTPBackend, StorageBackend

logger = logging.getLogger(__name__)


class RemoteStorage:
    """Interface for interacting with remote storage solutions.

    Supports multiple backends (AWS S3, Google Cloud, SFTP) through a unified interface.

    Attributes:
        storage_type: Type of remote storage (s3, gcs, sftp, local)
        is_connected: Boolean indicating connection status
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize remote storage interface.

        Args:
            config: Configuration dictionary with keys:
                - type: Storage type (s3, gcs, sftp, local)
                - enabled: Whether remote storage is enabled
                - bucket/path: Storage location details
                - credentials: Authentication information
        """
        self.config = config
        self.storage_type = config.get("type", "local")
        self.enabled = config.get("enabled", False)
        self.is_connected = False
        self.backend = None

        if self.enabled:
            self.backend = self._get_backend()

        logger.info(f"Remote storage initialized (type: {self.storage_type})")

    def connect(self) -> bool:
        """Establish connection to remote storage.

        Returns:
            True if connection successful, False otherwise
        """
        if not self.enabled:
            logger.info("Remote storage disabled")
            return True

        try:
            if self.backend:
                self.is_connected = self.backend.connect()
            logger.info(f"Connected to {self.storage_type} storage")
            return self.is_connected
        except Exception as e:
            logger.error(f"Failed to connect to remote storage: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from remote storage."""
        if self.backend and self.is_connected:
            self.backend.disconnect()
            self.is_connected = False
            logger.info("Disconnected from remote storage")

    def upload_file(self, local_path: str, remote_path: str) -> bool:
        """Upload file to remote storage.

        Args:
            local_path: Local file path
            remote_path: Remote storage path

        Returns:
            True if upload successful
        """
        if not self.enabled or not self.is_connected:
            logger.warning("Remote storage not available")
            return False

        try:
            if self.backend:
                self.backend.upload_file(local_path, remote_path)
            logger.info(f"Uploaded {local_path} to {remote_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to upload file: {e}")
            return False

    def download_file(self, remote_path: str, local_path: str) -> bool:
        """Download file from remote storage.

        Args:
            remote_path: Remote storage path
            local_path: Local file path for download

        Returns:
            True if download successful
        """
        if not self.enabled or not self.is_connected:
            logger.warning("Remote storage not available")
            return False

        try:
            if self.backend:
                self.backend.download_file(remote_path, local_path)
            logger.info(f"Downloaded {remote_path} to {local_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to download file: {e}")
            return False

    def list_files(self, remote_path: str = "") -> Optional[list]:
        """List files in remote storage.

        Args:
            remote_path: Remote storage path to list

        Returns:
            List of filenames or None if failed
        """
        if not self.enabled or not self.is_connected:
            return None

        try:
            if self.backend:
                return self.backend.list_files(remote_path)
            return []
        except Exception as e:
            logger.error(f"Failed to list files: {e}")
            return None

    def _get_backend(self) -> Optional["StorageBackend"]:
        """Factory method to get storage backend.

        Returns:
            StorageBackend instance or None
        """
        storage_type = self.storage_type.lower()

        if storage_type == "s3":
            return S3Backend(self.config)
        elif storage_type == "sftp":
            return SFTPBackend(self.config)
        elif storage_type == "local":
            return LocalBackend(self.config)
        else:
            logger.warning(f"Unsupported storage type: {storage_type}")
            return None
