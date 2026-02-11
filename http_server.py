#!/usr/bin/env python3
"""
HTTP Server for RE-RASSOR Motor Controller
Runs OUTSIDE Docker container - listens for HTTP POST requests from motor_controller
"""

from flask import Flask, request, jsonify
from datetime import datetime
import logging
import argparse
import json

app = Flask(__name__)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Store latest commands for monitoring
latest_commands = {
    'wheel': None,
    'front_arm': None,
    'back_arm': None,
    'front_drum': None,
    'back_drum': None,
    'routine': None,
    'emergency_stop': None
}

# Command history
command_history = []
MAX_HISTORY = 100


def log_command(command_type, data):
    """Log command to history"""
    entry = {
        'type': command_type,
        'data': data,
        'received_at': datetime.now().isoformat()
    }
    command_history.append(entry)
    if len(command_history) > MAX_HISTORY:
        command_history.pop(0)


@app.route('/wheel_command', methods=['POST'])
def wheel_command():
    """Receive wheel movement commands"""
    try:
        data = request.get_json()
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        timestamp = data.get('timestamp', 0.0)
        
        latest_commands['wheel'] = {
            'linear_x': linear_x,
            'angular_z': angular_z,
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat()
        }
        
        logger.info(f"WHEEL: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")
        log_command('wheel', data)
        
        # Here you would send actual commands to the robot hardware
        # For now, just acknowledge receipt
        
        return jsonify({
            'status': 'success',
            'message': 'Wheel command received',
            'linear_x': linear_x,
            'angular_z': angular_z
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing wheel command: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/arm_command', methods=['POST'])
def arm_command():
    """Receive arm movement commands"""
    try:
        data = request.get_json()
        arm = data.get('arm', 'unknown')
        action = data.get('action', 0.0)
        timestamp = data.get('timestamp', 0.0)
        
        cmd_key = f'{arm}_arm'
        latest_commands[cmd_key] = {
            'action': action,
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat()
        }
        
        action_str = "RAISE" if action > 0 else "LOWER" if action < 0 else "STOP"
        logger.info(f"ARM: {arm.upper()} arm - {action_str} (action={action:.2f})")
        log_command('arm', data)
        
        return jsonify({
            'status': 'success',
            'message': f'{arm} arm command received',
            'arm': arm,
            'action': action
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing arm command: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/drum_command', methods=['POST'])
def drum_command():
    """Receive drum commands"""
    try:
        data = request.get_json()
        drum = data.get('drum', 'unknown')
        action = data.get('action', 0.0)
        timestamp = data.get('timestamp', 0.0)
        
        cmd_key = f'{drum}_drum'
        latest_commands[cmd_key] = {
            'action': action,
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat()
        }
        
        action_str = "DIG" if action > 0 else "DUMP" if action < 0 else "STOP"
        logger.info(f"DRUM: {drum.upper()} drum - {action_str} (action={action:.2f})")
        log_command('drum', data)
        
        return jsonify({
            'status': 'success',
            'message': f'{drum} drum command received',
            'drum': drum,
            'action': action
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing drum command: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/routine_command', methods=['POST'])
def routine_command():
    """Receive routine commands"""
    try:
        data = request.get_json()
        routine = data.get('routine', 0)
        timestamp = data.get('timestamp', 0.0)
        
        latest_commands['routine'] = {
            'routine': routine,
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat()
        }
        
        routine_names = {
            0b000001: "AUTO_DRIVE",
            0b000010: "AUTO_DIG",
            0b000100: "AUTO_DUMP",
            0b001000: "AUTO_DOCK",
            0b010000: "FULL_AUTONOMY",
            0b100000: "STOP"
        }
        
        routine_name = routine_names.get(routine, f"UNKNOWN(0b{routine:06b})")
        logger.info(f"ROUTINE: {routine_name} (code={routine})")
        log_command('routine', data)
        
        return jsonify({
            'status': 'success',
            'message': 'Routine command received',
            'routine': routine,
            'routine_name': routine_name
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing routine command: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    """Receive emergency stop command"""
    try:
        data = request.get_json()
        timestamp = data.get('timestamp', 0.0)
        
        latest_commands['emergency_stop'] = {
            'timestamp': timestamp,
            'received_at': datetime.now().isoformat()
        }
        
        logger.warning("!!! EMERGENCY STOP ACTIVATED !!!")
        log_command('emergency_stop', data)
        
        # Here you would send emergency stop to all robot systems
        
        return jsonify({
            'status': 'success',
            'message': 'Emergency stop executed'
        }), 200
        
    except Exception as e:
        logger.error(f"Error processing emergency stop: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400


@app.route('/status', methods=['GET'])
def status():
    """Get server status and latest commands"""
    return jsonify({
        'status': 'running',
        'latest_commands': latest_commands,
        'command_count': len(command_history),
        'server_time': datetime.now().isoformat()
    }), 200


@app.route('/history', methods=['GET'])
def history():
    """Get command history"""
    limit = request.args.get('limit', 50, type=int)
    return jsonify({
        'history': command_history[-limit:],
        'total_commands': len(command_history)
    }), 200


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({'status': 'healthy'}), 200


def main():
    parser = argparse.ArgumentParser(description='HTTP Server for RE-RASSOR Motor Controller')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5001, help='Port to listen on (default: 5001)')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()
    
    logger.info('=' * 70)
    logger.info('RE-RASSOR Motor Controller HTTP Server')
    logger.info('=' * 70)
    logger.info(f'Listening on http://{args.host}:{args.port}')
    logger.info('Press Ctrl+C to stop')
    logger.info('=' * 70)
    logger.info('\nEndpoints:')
    logger.info('  POST /wheel_command    - Wheel movement commands')
    logger.info('  POST /arm_command      - Arm movement commands')
    logger.info('  POST /drum_command     - Drum commands')
    logger.info('  POST /routine_command  - Routine commands')
    logger.info('  POST /emergency_stop   - Emergency stop')
    logger.info('  GET  /status           - Server status')
    logger.info('  GET  /history          - Command history')
    logger.info('  GET  /health           - Health check')
    logger.info('=' * 70)
    
    try:
        app.run(host=args.host, port=args.port, debug=args.debug)
    except KeyboardInterrupt:
        logger.info('\nShutting down server...')


if __name__ == '__main__':
    main()