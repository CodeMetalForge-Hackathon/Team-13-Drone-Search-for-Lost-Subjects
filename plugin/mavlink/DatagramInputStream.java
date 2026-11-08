package com.atakmap.android.plugintemplate.plugin.mavlink;

import java.io.IOException;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;

/**
 * Presents a UDP socket as a byte stream for the dronefleet packet reader,
 * remembering who sent the most recent datagram.
 */
final class DatagramInputStream extends InputStream {

    private final DatagramSocket socket;
    private final byte[] buf = new byte[4096];
    private final DatagramPacket packet = new DatagramPacket(buf, buf.length);
    private int pos;
    private int len;
    private volatile InetSocketAddress lastSender;

    DatagramInputStream(DatagramSocket socket) {
        this.socket = socket;
    }

    InetSocketAddress getLastSender() {
        return lastSender;
    }

    @Override
    public int read() throws IOException {
        if (pos >= len)
            fill();
        return buf[pos++] & 0xFF;
    }

    @Override
    public int read(byte[] b, int off, int n) throws IOException {
        if (n == 0)
            return 0;
        if (pos >= len)
            fill();
        int c = Math.min(n, len - pos);
        System.arraycopy(buf, pos, b, off, c);
        pos += c;
        return c;
    }

    @Override
    public int available() {
        return len - pos;
    }

    private void fill() throws IOException {
        do {
            packet.setLength(buf.length);
            socket.receive(packet);
            len = packet.getLength();
            pos = 0;
            lastSender = (InetSocketAddress) packet.getSocketAddress();
        } while (len <= 0);
    }
}
