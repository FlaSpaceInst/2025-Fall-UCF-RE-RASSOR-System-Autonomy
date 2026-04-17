/**
 * SshTerminal.web.js
 * ──────────────────
 * SSH terminal screen for RE-RASSOR (Electron only).
 *
 * Connection sequence (tries each in order until one succeeds):
 *   1. ubuntu@shadow
 *   2. ubuntu@shadow.local
 *   3. ubuntu@<server IP from AsyncStorage, port stripped>
 *
 * Requires the SSH IPC bridge exposed by preload.js:
 *   window.api.sshConnect(target)
 *   window.api.sshInput(data)
 *   window.api.sshDisconnect()
 *   window.api.onSshData(cb)
 *   window.api.onSshClosed(cb)
 *   window.api.removeSshListeners()
 */

import React, { useState, useEffect, useRef } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView, TextInput } from 'react-native';
import { useFonts } from 'expo-font';
import AsyncStorage from '@react-native-async-storage/async-storage';

// Strip common ANSI / VT100 escape sequences so output is legible as plain text.
const ANSI_RE = /\x1B\[[0-9;]*[mGKHFJABCDsu]|\x1B\][^\x07]*\x07|\x1B[@-_][0-?]*[ -/]*[@-~]/g;
const stripAnsi = (s) => s.replace(ANSI_RE, '').replace(/\r\n/g, '\n').replace(/\r/g, '\n');

// Strip port from a stored "host:port" string, return just the host.
const hostOnly = (ipPort) => (ipPort || '').split(':')[0].trim();

// ── Component ──────────────────────────────────────────────────────────────────
export default function SshTerminal({ navigation }) {
    const [output,        setOutput]        = useState('');
    const [input,         setInput]         = useState('');
    const [status,        setStatus]        = useState('Initialising…');
    const [connState,     setConnState]     = useState('idle'); // idle|connecting|live|ended|failed
    const [currentTarget, setCurrentTarget] = useState('');

    // Queue of targets to try, managed as a ref so IPC callbacks see current state.
    const queueRef  = useRef([]);   // remaining targets to try
    const liveRef   = useRef(false); // true once we receive SSH data (session established)
    const scrollRef = useRef(null);

    const [fontsLoaded] = useFonts({ NasaFont: require('../../../assets/nasa.ttf') });

    // ── Append text to terminal output ────────────────────────────────────────
    const appendOutput = (text) => {
        setOutput((prev) => prev + stripAnsi(text));
    };

    // ── Attempt connection to the next target in the queue ───────────────────
    const tryNext = (queue, overrideTarget) => {
        const target = overrideTarget ?? queue.shift();
        if (!target) {
            setStatus('All connection attempts failed');
            setConnState('failed');
            return;
        }
        queueRef.current = queue;
        liveRef.current  = false;
        setCurrentTarget(target);
        setConnState('connecting');
        setStatus(`Connecting to ${target}…`);
        appendOutput(`\n--- Attempting ${target} ---\n`);

        if (window.api?.sshConnect) {
            window.api.sshConnect(target);
        } else {
            appendOutput('[ERROR] SSH API not available — is this running inside Electron?\n');
            setStatus('SSH API unavailable');
            setConnState('failed');
        }
    };

    // ── Build target list and start connecting ────────────────────────────────
    const startSession = async () => {
        const stored   = await AsyncStorage.getItem('myIp');
        const serverIp = hostOnly(stored);

        // Build the ordered list; include IP fallback only if it's a real address
        const targets = [
            'ubuntu@shadow.local',
            'ubuntu@shadow',
            ...(serverIp ? [`ubuntu@${serverIp}`] : []),
        ];

        const [first, ...rest] = targets;
        tryNext(rest, first);
    };

    // ── IPC listeners — set up once on mount ─────────────────────────────────
    useEffect(() => {
        window.api?.onSshData?.((data) => {
            appendOutput(data);
            if (!liveRef.current) {
                liveRef.current = true;
                setConnState('live');
                setStatus('Connected');
            }
        });

        window.api?.onSshClosed?.(({ code, target }) => {
            appendOutput(`\n--- ${target} exited (code ${code ?? '?'}) ---\n`);

            if (!liveRef.current && code !== 0) {
                // Failed before session was established — try the next target
                tryNext(queueRef.current);
            } else {
                const ok = code === 0;
                setConnState(ok ? 'ended' : 'failed');
                setStatus(ok ? 'Session ended' : 'Connection failed');
            }
        });

        startSession();

        return () => {
            window.api?.removeSshListeners?.();
            window.api?.sshDisconnect?.();
        };
    }, []);

    // ── Auto-scroll terminal output ───────────────────────────────────────────
    useEffect(() => {
        scrollRef.current?.scrollToEnd({ animated: false });
    }, [output]);

    // ── Send input to SSH stdin ───────────────────────────────────────────────
    const handleSend = () => {
        const toSend = input;
        setInput('');
        appendOutput(`${toSend}\n`);
        window.api?.sshInput?.(toSend + '\n');
    };

    // ── Status dot colour ─────────────────────────────────────────────────────
    const dotColor =
        connState === 'live'       ? '#00c853' :
        connState === 'failed'     ? '#f44336' :
        connState === 'connecting' ? '#ffa000' : '#555';

    if (!fontsLoaded) return <View style={styles.root} />;

    // ── Render ────────────────────────────────────────────────────────────────
    return (
        <View style={styles.root}>
            {/* ── Header ────────────────────────────────────────────────────── */}
            <View style={styles.header}>
                <Pressable style={styles.backBtn} onPress={() => navigation.goBack()}>
                    <Text style={styles.backBtnText}>← BACK</Text>
                </Pressable>
                <Text style={styles.headerTitle}>SSH TERMINAL</Text>
                <View style={styles.headerRight}>
                    <View style={[styles.dot, { backgroundColor: dotColor }]} />
                    <Text style={styles.headerIp}>{currentTarget || '—'}</Text>
                </View>
            </View>

            {/* ── Status bar ────────────────────────────────────────────────── */}
            <View style={styles.statusBar}>
                <Text style={styles.statusText}>{status}</Text>
            </View>

            {/* ── Terminal output ───────────────────────────────────────────── */}
            <ScrollView
                ref={scrollRef}
                style={styles.terminal}
                contentContainerStyle={styles.terminalContent}
            >
                <Text style={styles.termText} selectable>{output}</Text>
            </ScrollView>

            {/* ── Input row ─────────────────────────────────────────────────── */}
            <View style={styles.inputRow}>
                <Text style={styles.prompt}>$</Text>
                <TextInput
                    style={styles.inputField}
                    value={input}
                    onChangeText={setInput}
                    onSubmitEditing={handleSend}
                    placeholder="Enter command…"
                    placeholderTextColor="#444"
                    returnKeyType="send"
                    autoCorrect={false}
                    autoCapitalize="none"
                    blurOnSubmit={false}
                />
                <Pressable style={styles.sendBtn} onPress={handleSend}>
                    <Text style={styles.actionBtnText}>SEND</Text>
                </Pressable>
                {(connState === 'ended' || connState === 'failed') && (
                    <Pressable
                        style={styles.reconnectBtn}
                        onPress={() => {
                            setOutput('');
                            startSession();
                        }}
                    >
                        <Text style={styles.actionBtnText}>RECONNECT</Text>
                    </Pressable>
                )}
            </View>
        </View>
    );
}

// ── Styles ─────────────────────────────────────────────────────────────────────
const DARK   = '#1a1d1d';
const PANEL  = '#232727';
const BORDER = '#383d3d';
const TEXT   = '#e0e0e0';
const DIM    = '#888';
const NASA   = 'NasaFont';

const styles = StyleSheet.create({
    root:          { flex: 1, backgroundColor: DARK, flexDirection: 'column' },

    // Header — identical to AutonomyController
    header:        { flexDirection: 'row', alignItems: 'center', backgroundColor: '#111',
                     paddingHorizontal: 16, paddingVertical: 10, borderBottomWidth: 1,
                     borderBottomColor: BORDER },
    backBtn:       { paddingHorizontal: 12, paddingVertical: 6, backgroundColor: '#333',
                     borderRadius: 4, marginRight: 16 },
    backBtnText:   { color: TEXT, fontSize: 13, fontWeight: '600', fontFamily: NASA },
    headerTitle:   { flex: 1, color: '#fff', fontSize: 18, fontWeight: '700',
                     letterSpacing: 2, textAlign: 'center', fontFamily: NASA },
    headerRight:   { flexDirection: 'row', alignItems: 'center', minWidth: 160,
                     justifyContent: 'flex-end' },
    dot:           { width: 8, height: 8, borderRadius: 4, marginRight: 8 },
    headerIp:      { color: DIM, fontSize: 12 },

    // Status bar
    statusBar:     { backgroundColor: '#191c1c', paddingHorizontal: 16, paddingVertical: 4,
                     borderBottomWidth: 1, borderBottomColor: BORDER },
    statusText:    { color: DIM, fontSize: 12 },

    // Terminal output area
    terminal:      { flex: 1, backgroundColor: '#0d1117' },
    terminalContent: { padding: 12, flexGrow: 1 },
    termText:      { color: '#c9d1d9', fontSize: 13, fontFamily: 'monospace', lineHeight: 20 },

    // Input bar at bottom
    inputRow:      { flexDirection: 'row', alignItems: 'center', backgroundColor: PANEL,
                     borderTopWidth: 1, borderTopColor: BORDER,
                     paddingHorizontal: 8, paddingVertical: 8, gap: 8 },
    prompt:        { color: '#00c853', fontSize: 14, fontFamily: 'monospace',
                     paddingHorizontal: 4 },
    inputField:    { flex: 1, backgroundColor: '#0d1117', color: TEXT, fontSize: 13,
                     fontFamily: 'monospace', paddingHorizontal: 10, paddingVertical: 6,
                     borderRadius: 4, borderWidth: 1, borderColor: BORDER },
    sendBtn:       { backgroundColor: '#1565c0', borderRadius: 4,
                     paddingVertical: 8, paddingHorizontal: 16 },
    reconnectBtn:  { backgroundColor: '#b71c1c', borderRadius: 4,
                     paddingVertical: 8, paddingHorizontal: 16 },
    actionBtnText: { color: '#fff', fontWeight: '700', fontSize: 12,
                     letterSpacing: 1, fontFamily: NASA },
});
