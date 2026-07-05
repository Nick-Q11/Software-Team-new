import time
import requests
import threading
import asyncio
import aiohttp

class TelemetryClient:
    def __init__(self, server_ip="127.0.0.1", server_port=5000):
        self.url = f'http://{server_ip}:{server_port}/update_position'
        self.latitude = 0.0
        self.longitude = 0.0
        self.running = False
        self._session = None
        self._task = None
        
    def update_location(self, lat, lon):
        """Aktualisiert die Koordinaten im Objekt"""
        self.latitude = float(lat)
        self.longitude = float(lon)
        
    async def send_position_loop(self):
        """Sendet die Daten im Hintergrund an den Server"""
        # KORREKTUR: Nutzt 'async with', damit die Session garantiert initialisiert ist
        async with aiohttp.ClientSession() as session:
            self._session = session
            
            try:
                while self.running:
                    data = {
                        'latitude': self.latitude,
                        'longitude': self.longitude
                    }
                    try:
                        async with self._session.post(self.url, json=data, timeout=1.0) as response:
                            if response.status == 200:
                                print(f"[Gesendet] {self.latitude}, {self.longitude}")
                            else:
                                print(f"Server-Fehler. Status: {response.status}")
                    except Exception as e:
                        print(f"Keine Verbindung zum Webserver: {e}")
                    
                    # Wartet 1 Sekunde vor dem nächsten Senden
                    await asyncio.sleep(1)
            finally:
                # Setzt die Referenz beim Verlassen wieder zurück
                self._session = None
        
    def start(self):
        """Startet die Hintergrund-Schleife als asyncio-Task"""
        if not self.running:
            self.running = True
            # Erstellt einen asynchronen Hintergrund-Task im bestehenden Event-Loop
            self._task = asyncio.create_task(self.send_position_loop())
            print("Telemetry Client (Async) gestartet.")
        
    def stop(self):
        """Stoppt die Schleife und räumt den Task auf"""
        if self.running:
            self.running = False
            if self._task:
                self._task.cancel()
            print("Telemetry Client gestoppt.")