/**
 * websocket-service.js
 * ─────────────────────
 * Thin wrapper around socket.io-client for connecting to the
 * re_rassor_controller_server backend (port 5000).
 *
 * Usage:
 *   import ws from './websocket-service';
 *   ws.connect('192.168.1.2:5000');
 *   ws.on('odom', (data) => { ... });
 *   ws.emit('subscribe_odom');
 *   ws.disconnect();
 */

import { io } from 'socket.io-client';

class WebSocketService {
    constructor() {
        this._socket = null;
        this._host   = null;
    }

    /** Connect to the controller server at the given host string (e.g. "192.168.1.2:5000"). */
    connect(host) {
        if (this._socket) this.disconnect();

        this._host = host;
        const url = host.startsWith('http') ? host : `http://${host}`;

        this._socket = io(url, {
            transports:       ['websocket'],
            reconnection:     true,
            reconnectionDelay: 2000,
            timeout:          5000,
        });

        return this._socket;
    }

    /** Register a listener for a server-emitted event. */
    on(event, callback) {
        if (!this._socket) return;
        this._socket.on(event, callback);
    }

    /** Remove a listener. */
    off(event, callback) {
        if (!this._socket) return;
        this._socket.off(event, callback);
    }

    /** Emit an event to the server. */
    emit(event, data) {
        if (!this._socket) return;
        this._socket.emit(event, data);
    }

    get connected() {
        return this._socket?.connected ?? false;
    }

    get socket() {
        return this._socket;
    }

    disconnect() {
        if (this._socket) {
            this._socket.disconnect();
            this._socket = null;
        }
        this._host = null;
    }
}

// Export a singleton so all screens share one connection when on the same IP.
export default new WebSocketService();
