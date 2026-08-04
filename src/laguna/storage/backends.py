"""Storage backend implementations for RemoteStorage."""

import logging
from typing import Any, Dict

logger = logging.getLogger(__name__)


class StorageBackend:
    """Abstract base class for storage backends."""

    def __init__(self, config: Dict[str, Any]):
        """Initialize storage backend.

        Args:
            config: Backend configuration
        """
        self.config = config

    def connect(self) -> bool:
        """Establish connection to storage."""
        raise NotImplementedError

    def disconnect(self) -> None:
        """Close connection to storage."""
        raise NotImplementedError

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload file."""
        raise NotImplementedError

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download file."""
        raise NotImplementedError

    def list_files(self, remote_path: str) -> list:
        """List files in storage."""
        raise NotImplementedError


class S3Backend(StorageBackend):
    """AWS S3 storage backend."""

    def connect(self) -> bool:
        """Connect to S3."""
        # TODO: Implement S3 connection using boto3
        logger.info("S3 backend initialized (stub)")
        return True

    def disconnect(self) -> None:
        """Close S3 connection."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload to S3."""
        # TODO: Implement S3 upload
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download from S3."""
        # TODO: Implement S3 download
        pass

    def list_files(self, remote_path: str) -> list:
        """List S3 files."""
        # TODO: Implement S3 list
        return []


class SFTPBackend(StorageBackend):
    """SFTP remote storage backend."""

    def connect(self) -> bool:
        """Connect via SFTP."""
        # TODO: Implement SFTP connection using paramiko
        logger.info("SFTP backend initialized (stub)")
        return True

    def disconnect(self) -> None:
        """Close SFTP connection."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload via SFTP."""
        # TODO: Implement SFTP upload
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download via SFTP."""
        # TODO: Implement SFTP download
        pass

    def list_files(self, remote_path: str) -> list:
        """List SFTP files."""
        # TODO: Implement SFTP list
        return []


class LocalBackend(StorageBackend):
    """Local filesystem storage backend."""

    def connect(self) -> bool:
        """Connect to local storage (always available)."""
        return True

    def disconnect(self) -> None:
        """Disconnect from local storage."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Copy file to local storage."""
        # TODO: Implement local file copy
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Copy file from local storage."""
        # TODO: Implement local file copy
        pass

    def list_files(self, remote_path: str) -> list:
        """List local files."""
        # TODO: Implement local directory listing
        return []
