"""Main application orchestrator."""

import asyncio
import logging
import signal
from pathlib import Path
from typing import Optional

import uvicorn

from .config.settings import Settings, load_settings, save_settings
from .core.events import EventBus, EventType, event_bus
from .core.models import BallCounts
from .gpio.ball_counter import BallCounter
from .gpio.channel_monitor import ChannelMonitor
from .led.controller import LEDController
from .network.nt_client import NetworkTablesClient
from .web.routes.api import broadcast_counts, broadcast_nt_status, set_dependencies
from .web.server import create_app

logger = logging.getLogger(__name__)


class HubCounterApp:
    """Main application orchestrating all components."""

    def __init__(self, config_path: Optional[Path] = None):
        """Initialize the application.

        Args:
            config_path: Path to configuration file.
        """
        self._config_path = config_path or Path("config.yaml")
        self._settings = load_settings(self._config_path)

        # Components
        self._event_bus = event_bus
        self._ball_counter = BallCounter(num_channels=4)
        self._channel_monitor: Optional[ChannelMonitor] = None
        self._led_controller: Optional[LEDController] = None
        self._nt_client: Optional[NetworkTablesClient] = None

        # State
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._shutdown_event = asyncio.Event()
        self._server: Optional[uvicorn.Server] = None

        logger.info("HubCounterApp initialized")

    def _setup_components(self) -> None:
        """Initialize all hardware and network components."""
        # Set up event loop for sync-to-async bridging
        self._loop = asyncio.get_event_loop()
        self._event_bus.set_loop(self._loop)

        # Initialize channel monitor
        self._channel_monitor = ChannelMonitor(
            config=self._settings.gpio,
            event_bus=self._event_bus,
            on_ball_detected=self._on_ball_detected,
        )

        # Initialize LED controller
        self._led_controller = LEDController(
            led_config=self._settings.led,
            threshold_config=self._settings.thresholds,
            color_config=self._settings.colors,
        )

        # Initialize NetworkTables client
        self._nt_client = NetworkTablesClient(
            config=self._settings.network,
            event_bus=self._event_bus,
        )

        # Register ball counter callback for event bus updates
        self._ball_counter.on_count_change(self._on_count_changed)

        # Subscribe to NT connection changes for WebSocket broadcasting
        self._event_bus.subscribe(
            EventType.NT_CONNECTED, self._on_nt_connection_changed
        )

        logger.info("All components initialized")

    def _on_ball_detected(self, channel: int) -> None:
        """Handle ball detection from GPIO.

        Args:
            channel: Channel where ball was detected.
        """
        counts = self._ball_counter.increment(channel)

        # Trigger LED pulse
        if self._led_controller and self._loop:
            self._led_controller.trigger_pulse(counts.total, self._loop)

        # Publish to NetworkTables
        if self._nt_client:
            self._nt_client.publish_counts(counts)

    def _on_count_changed(self, counts: BallCounts) -> None:
        """Handle count changes.

        Args:
            counts: Updated ball counts.
        """
        # Schedule WebSocket broadcast
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                broadcast_counts(counts), self._loop
            )

    async def _on_nt_connection_changed(self, payload: dict) -> None:
        """Handle NT connection state changes.

        Args:
            payload: Event payload with 'connected' bool.
        """
        connected = payload.get("connected", False)
        await broadcast_nt_status(connected)

    def _save_settings(self, settings: Settings) -> None:
        """Save settings to file.

        Args:
            settings: Settings to save.
        """
        self._settings = settings
        save_settings(settings, self._config_path)
        logger.info("Settings saved")

    async def _run_server(self) -> None:
        """Run the web server."""
        app = create_app()

        # Set up API dependencies
        set_dependencies(
            ball_counter=self._ball_counter,
            nt_client=self._nt_client,
            led_controller=self._led_controller,
            settings=self._settings,
            save_settings_fn=self._save_settings,
        )

        config = uvicorn.Config(
            app,
            host=self._settings.web.host,
            port=self._settings.web.port,
            log_level="info",
        )
        self._server = uvicorn.Server(config)

        logger.info(
            f"Starting web server on {self._settings.web.host}:{self._settings.web.port}"
        )
        await self._server.serve()

    def _setup_signal_handlers(self) -> None:
        """Set up signal handlers for graceful shutdown."""

        def signal_handler(sig, frame):
            logger.info(f"Received signal {sig}, initiating shutdown...")
            self._shutdown_event.set()
            if self._server:
                self._server.should_exit = True

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    async def run(self) -> None:
        """Run the application."""
        self._setup_components()
        self._setup_signal_handlers()

        # Start hardware components
        if self._channel_monitor:
            self._channel_monitor.start()

        if self._led_controller:
            self._led_controller.start()

        if self._nt_client:
            self._nt_client.start()

        logger.info("Hub Counter started")

        try:
            # Run web server (this blocks until shutdown)
            await self._run_server()
        finally:
            await self.shutdown()

    async def shutdown(self) -> None:
        """Clean up and shut down all components."""
        logger.info("Shutting down...")

        # Stop hardware components
        if self._channel_monitor:
            self._channel_monitor.stop()

        if self._led_controller:
            self._led_controller.stop()

        if self._nt_client:
            self._nt_client.stop()

        logger.info("Hub Counter stopped")


def run_app(config_path: Optional[Path] = None) -> None:
    """Run the Hub Counter application.

    Args:
        config_path: Path to configuration file.
    """
    app = HubCounterApp(config_path)
    asyncio.run(app.run())
