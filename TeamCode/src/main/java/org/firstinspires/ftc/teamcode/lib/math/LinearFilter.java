package org.firstinspires.ftc.teamcode.lib.math;

/**
 * Basit Linear Filter (Low-pass / Moving Average) FTCLib ve Java için
 */
public class LinearFilter {

    private final int windowSize;       // Hareketli ortalama için pencere boyutu
    private final double[] window;      // Değerleri tutar
    private int index = 0;              // Döngüsel index
    private int count = 0;              // Kaç değer girildi
    private double sum = 0.0;           // Toplam değer
    private boolean useIIR = false;     // Tek kutuplu IIR mi?

    private double alpha = 0.0;         // IIR için alpha
    private double prev = 0.0;          // IIR önceki değer

    /**
     * Hareketli Ortalama filtresi
     */
    public LinearFilter(int windowSize) {
        this.windowSize = windowSize;
        this.window = new double[windowSize];
        this.useIIR = false;
    }

    /**
     * Tek kutuplu IIR filtresi
     * @param alpha 0..1, 1 hızlı tepki, 0 tamamen eski değer
     */
    public LinearFilter(double alpha) {
        this.alpha = alpha;
        this.useIIR = true;
        this.windowSize = 1;
        this.window = new double[1];
    }

    /**
     * Yeni değeri filtreler ve döndürür
     */
    public double calculate(double value) {
        if (useIIR) {
            prev = alpha * value + (1 - alpha) * prev;
            return prev;
        } else {
            if (count < windowSize) count++;
            sum -= window[index];
            window[index] = value;
            sum += value;
            index = (index + 1) % windowSize;
            return sum / count;
        }
    }

    /**
     * Filtreyi sıfırlar
     */
    public void reset() {
        index = 0;
        count = 0;
        sum = 0.0;
        prev = 0.0;
        for (int i = 0; i < window.length; i++) window[i] = 0.0;
    }
}
