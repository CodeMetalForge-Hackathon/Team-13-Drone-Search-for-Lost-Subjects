package com.atakmap.android.plugintemplate.plugin.ui;

import android.os.Handler;
import android.os.Looper;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Runs one flight sequence at a time on a worker thread. {@link #cancelAll()}
 * interrupts it (used by R2Home, Land and Kill), which aborts any blocking wait.
 */
public class SequenceRunner {

    public interface Logger {
        /** Called on the main thread. */
        void log(String line);
    }

    public interface Condition {
        boolean met();
    }

    public interface Task {
        void run(Ctx ctx) throws Exception;
    }

    public final class Ctx {
        private final Logger logger;

        Ctx(Logger logger) {
            this.logger = logger;
        }

        public void log(String line) {
            main.post(() -> logger.log(line));
        }

        public void sleep(long ms) throws InterruptedException {
            Thread.sleep(ms);
        }

        /** Wait for a condition; throws on timeout. */
        public void waitFor(Condition c, long timeoutMs, String what) throws Exception {
            if (!waitForOptional(c, timeoutMs))
                throw new Exception(what + " timed out");
        }

        /** Wait for a condition; returns false on timeout. */
        public boolean waitForOptional(Condition c, long timeoutMs) throws InterruptedException {
            long deadline = System.currentTimeMillis() + timeoutMs;
            while (System.currentTimeMillis() < deadline) {
                if (c.met())
                    return true;
                Thread.sleep(100);
            }
            return c.met();
        }
    }

    private final ExecutorService exec = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "KoalifiedSequence");
        t.setDaemon(true);
        return t;
    });
    private final Handler main = new Handler(Looper.getMainLooper());
    private volatile Future<?> current;
    private volatile String currentName;

    public synchronized boolean start(String name, Logger logger, Task task,
            Runnable onFinished) {
        if (isBusy()) {
            logger.log("Busy: " + currentName + " is still running");
            return false;
        }
        currentName = name;
        final Ctx ctx = new Ctx(logger);
        current = exec.submit(() -> {
            try {
                task.run(ctx);
                ctx.log("✓ " + name + " complete");
            } catch (InterruptedException e) {
                ctx.log("■ " + name + " cancelled");
            } catch (Exception e) {
                ctx.log("✗ " + name + ": " + e.getMessage());
            } finally {
                if (onFinished != null)
                    main.post(onFinished);
            }
        });
        return true;
    }

    public boolean isBusy() {
        Future<?> f = current;
        return f != null && !f.isDone();
    }

    public void cancelAll() {
        Future<?> f = current;
        if (f != null)
            f.cancel(true);
    }

    public void shutdown() {
        cancelAll();
        exec.shutdownNow();
    }
}
