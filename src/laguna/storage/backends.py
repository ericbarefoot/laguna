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
        """Establish connection to storage.

        Returns:
            True if connection successful, False otherwise.

        Raises:
            NotImplementedError: Subclasses must implement.
        """
        raise NotImplementedError

    def disconnect(self) -> None:
        """Close connection to storage.

        Raises:
            NotImplementedError: Subclasses must implement.
        """
        raise NotImplementedError

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload file to remote storage.

        Args:
            local_path: Path to local file.
            remote_path: Remote destination path.

        Raises:
            NotImplementedError: Subclasses must implement.
        """
        raise NotImplementedError

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download file from remote storage.

        Args:
            remote_path: Path on remote storage.
            local_path: Local destination path.

        Raises:
            NotImplementedError: Subclasses must implement.
        """
        raise NotImplementedError

    def list_files(self, remote_path: str) -> list:
        """List files in remote storage.

        Args:
            remote_path: Path to list on remote storage.

        Returns:
            List of filenames at the given path.

        Raises:
            NotImplementedError: Subclasses must implement.
        """
        raise NotImplementedError


class S3Backend(StorageBackend):
    """AWS S3 storage backend."""

    def connect(self) -> bool:
        """Establish connection to S3 bucket.

        Returns:
            True if connection successful, False otherwise.
        """
        # TODO: Implement S3 connection using boto3
        logger.info("S3 backend initialized (stub)")
        return True

    def disconnect(self) -> None:
        """Close S3 connection."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload file to S3 bucket.

        Args:
            local_path: Path to local file.
            remote_path: S3 object key (destination path).
        """
        # TODO: Implement S3 upload
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download file from S3 bucket.

        Args:
            remote_path: S3 object key to download.
            local_path: Local destination path.
        """
        # TODO: Implement S3 download
        pass

    def list_files(self, remote_path: str) -> list:
        """List objects in S3 bucket at prefix.

        Args:
            remote_path: S3 prefix (directory-like path).

        Returns:
            List of object keys at the given prefix.
        """
        # TODO: Implement S3 list
        return []


class SFTPBackend(StorageBackend):
    """SFTP remote storage backend."""

    def connect(self) -> bool:
        """Establish SFTP connection to remote server.

        Returns:
            True if connection successful, False otherwise.
        """
        # TODO: Implement SFTP connection using paramiko
        logger.info("SFTP backend initialized (stub)")
        return True

    def disconnect(self) -> None:
        """Close SFTP connection to remote server."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Upload file via SFTP.

        Args:
            local_path: Path to local file.
            remote_path: Remote destination path on SFTP server.
        """
        # TODO: Implement SFTP upload
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Download file via SFTP.

        Args:
            remote_path: Path on remote SFTP server.
            local_path: Local destination path.
        """
        # TODO: Implement SFTP download
        pass

    def list_files(self, remote_path: str) -> list:
        """List files on remote SFTP server.

        Args:
            remote_path: Path on remote SFTP server to list.

        Returns:
            List of filenames at the given remote path.
        """
        # TODO: Implement SFTP list
        return []


class LocalBackend(StorageBackend):
    """Local filesystem storage backend."""

    def connect(self) -> bool:
        """Establish connection to local storage.

        Returns:
            True (local storage is always available).
        """
        return True

    def disconnect(self) -> None:
        """Close local storage connection."""
        pass

    def upload_file(self, local_path: str, remote_path: str) -> None:
        """Copy file to local storage location.

        Args:
            local_path: Source file path.
            remote_path: Local destination path.
        """
        # TODO: Implement local file copy
        pass

    def download_file(self, remote_path: str, local_path: str) -> None:
        """Copy file from local storage location.

        Args:
            remote_path: Source path in local storage.
            local_path: Local destination path.
        """
        # TODO: Implement local file copy
        pass

    def list_files(self, remote_path: str) -> list:
        """List files in local directory.

        Args:
            remote_path: Local directory path to list.

        Returns:
            List of filenames in the directory.
        """
        # TODO: Implement local directory listing
        return []
