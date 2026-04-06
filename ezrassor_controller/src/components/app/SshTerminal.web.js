/**
 * SshTerminal.web.js
 * ──────────────────
 * SSH terminal screen for RE-RASSOR (Electron only).
 *
 * Connection sequence:
 *   1. Attempt  ubuntu@shadow.local  (ConnectTimeout = 5 s)
 *   2. On non-zero exit before a live session, fall back to  ubuntu@shadow
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

const PRIMARY_TARGET  = 'ubuntu@shadow.local';
const FALLBACK_TARGET = 'ubuntu@shadow';

// Strip common ANSI / VT100 escape sequences so output is legible in plain text.
const ANSI_RE = /\x1B\[[0-9;]*[mGKHFJABCDsu]|\x1B\][^\x07]*\x07|\x1B[@-_][0-?]*[ -/]*[@-~]/g;
const stripAnsi = (s) => s.replace(ANSI_RE, '').replace(/\r\n/g, '\n').replace(/\r/g, '\n');

// ── Component ──────────────────────────────────────────────────────────────────
export default function SshTerminal({ navigation }) {
    const [output,        setOutput]        = useState('');
    const [input,         setInput]         = useState('');
    const [status,        setStatus]        = useState('Initialising…');
    const [connState,     setConnState]     = useState('idle');   // idle|connecting|live|ended|failed
    const [currentTarget, setCurrentTarget] = useState(PRIMARY_TARGET);

    // phaseRef tracks the connection phase for use inside async IPC callbacks
    // where React state would be stale.
    // Values: 'primary' | 'fallback' | 'live' | 'ended' | 'failed'
    const phaseRef  = useRef('idle');
    const scrollRef = useRef(null);

    const [fontsLoaded] = useFonts({ NasaFont: require('../../../assets/nasa.ttf') });

    // ── Helpers ───────────────────────────────────────────────────────────────
    const appendOutput = (text) => {
        setOutput((prev) => prev + stripAnsi(text));
    };

    const connect = (target) => {
        const isPrimary = target === PRIMARY_TARGET;
        phaseRef.current = isPrimary ? 'primary' : 'fallback';
        setConnState('connecting');
        setCurrentTarget(target);
        setStatus(`Connecting to ${target}…`);
        appendOutput(`\n--- Attempting ${target} ---\n`);

        if (window.api?.sshConnect) {
            window.api.sshConnect(target);
        } else {
            appendOutput('[ERROR] SSH API not available — is this running inside Electron?\n');
            setStatus('SSH API unavailable');
            setConnState('failed');
            phaseRef.current = 'failed';
        }
    };

    // ── IPC listeners (set up once on mount) ──────────────────────────────────
    useEffect(() => {
        window.api?.onSshData?.((data) => {
            appendOutput(data);
            if (phaseRef.current === 'primary' || phaseRef.current === 'fallback') {
                phaseRef.current = 'live';
                setConnState('live');
                setStatus('Connected');
            }
        });

        window.api?.onSshClosed?.(({ code, target }) => {
            appendOutput(`\n--- ${target} exited (code ${code ?? '?'}) ---\n`);

            if (phaseRef.current === 'primary' && code !== 0) {
                // Primary failed before establishing a session → try fallback
                connect(FALLBACK_TARGET);
            } else {
                const ok = code === 0;
                phaseRef.current = ok ? 'ended' : 'failed';
                setConnState(ok ? 'ended' : 'failed');
                setStatus(ok ? 'Session ended' : 'Connection failed');
            }
        });

        connect(PRIMARY_TARGET);

        return () => {
            window.api?.removeSshListeners?.();
            window.api?.sshDisconnect?.();
        };
    }, []);

    // ── Auto-scroll when output grows ─────────────────────────────────────────
    useEffect(() => {
        scrollRef.current?.scrollToEnd({ animated: false });
    }, [output]);

    // ── Send command to SSH stdin ─────────────────────────────────────────────
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
                    <Text style={styles.headerIp}>{currentTarget}</Text>
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
                            connect(PRIMARY_TARGET);
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

    // Terminal output area — dark like a real terminal
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
