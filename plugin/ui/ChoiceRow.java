package com.atakmap.android.plugintemplate.plugin.ui;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.util.TypedValue;
import android.view.Gravity;
import android.widget.Button;
import android.widget.LinearLayout;

import java.util.ArrayList;
import java.util.List;

/**
 * A row of side-by-side options with the chosen one highlighted, so every option and the default
 * are visible at a glance (instead of a button that cycles through hidden values).
 */
final class ChoiceRow<T> {

    interface OnPick<T> {
        void picked(T value);
    }

    private static final int ON_FILL = Color.parseColor("#1565C0");
    private static final int ON_STROKE = Color.parseColor("#64B5F6");
    private static final int OFF_FILL = Color.parseColor("#2B2B2B");
    private static final int OFF_STROKE = Color.parseColor("#5A5A5A");
    private static final int OFF_TEXT = Color.parseColor("#BDBDBD");

    private final List<T> values;
    private final List<Button> buttons = new ArrayList<>();
    private T selected;

    ChoiceRow(LinearLayout row, List<T> values, List<String> labels, T selected,
            final OnPick<T> onPick) {
        this.values = values;
        this.selected = selected;
        Context ctx = row.getContext();
        row.removeAllViews();
        row.setOrientation(LinearLayout.HORIZONTAL);
        for (int i = 0; i < values.size(); i++) {
            final T v = values.get(i);
            Button b = new Button(ctx);
            b.setText(labels.get(i));
            b.setAllCaps(false);
            b.setGravity(Gravity.CENTER);
            b.setMinHeight(dp(ctx, 44));
            b.setPadding(dp(ctx, 4), dp(ctx, 6), dp(ctx, 4), dp(ctx, 6));
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0,
                    LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
            int m = dp(ctx, 2);
            lp.setMargins(m, m, m, m);
            b.setOnClickListener(x -> {
                select(v);
                onPick.picked(v);
            });
            row.addView(b, lp);
            buttons.add(b);
        }
        select(selected);
    }

    T selected() {
        return selected;
    }

    void select(T v) {
        selected = v;
        for (int i = 0; i < buttons.size(); i++)
            style(buttons.get(i), values.get(i).equals(v));
    }

    void setEnabled(boolean enabled) {
        for (Button b : buttons) {
            b.setEnabled(enabled);
            b.setAlpha(enabled ? 1f : 0.45f);
        }
    }

    private static void style(Button b, boolean on) {
        Context ctx = b.getContext();
        GradientDrawable bg = new GradientDrawable();
        bg.setCornerRadius(dp(ctx, 4));
        bg.setColor(on ? ON_FILL : OFF_FILL);
        bg.setStroke(dp(ctx, on ? 2 : 1), on ? ON_STROKE : OFF_STROKE);
        b.setBackground(bg);
        b.setTextColor(on ? Color.WHITE : OFF_TEXT);
        b.setTypeface(null, on ? Typeface.BOLD : Typeface.NORMAL);
    }

    private static int dp(Context ctx, int v) {
        return (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v,
                ctx.getResources().getDisplayMetrics());
    }
}
