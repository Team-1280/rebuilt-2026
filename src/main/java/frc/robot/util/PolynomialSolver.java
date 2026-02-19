package frc.robot.util;

public final class PolynomialSolver {
    private PolynomialSolver() {}

    /**
     * Get 0, 2, or 4 real roots from the given depressed quartic.
     *
     * @param a quartic coefficient, not equal to 0
     * @param c quadratic coefficient
     * @param d linear coefficient
     * @param e constant coefficient
     * @return an array of the real roots, of length 0, 2, or 4
     */
    public static double[] depressedQuarticRealRoots(double a, double c, double d, double e) {
        // Ferrari's method
        // Normalize coefficients
        double p = c / a;
        double q = d / a;
        double r = e / a;
        // Solve the resolvent cubic for a real root
        double A = 8;
        double B = -4 * p;
        double C = -8 * r;
        double D = 4 * p * r - q * q;
        double y1 =
                cubicRealRoots(A, B, C, D)[0]; // Chooses the only real root or the principal root
        // Solve the two quadratics
        double u;
        double v;
        if (2 * y1 - p > 1e-12) {
            u = Math.sqrt(2 * y1 - p);
            v = q / (2 * u);
        } else {
            // Biquadratic case: d close to 0 and real roots exist
            u = 0.0;
            v = Math.sqrt(y1 * y1 - r);
        }
        double[] quadRoots1 = quadraticRealRoots(1.0, u, y1 - v);
        double[] quadRoots2 = quadraticRealRoots(1.0, -u, y1 + v);
        // Add together the solutions
        if (quadRoots2.length == 0) {
            return quadRoots1;
        }
        if (quadRoots1.length == 0) {
            return quadRoots2;
        }
        return new double[] {quadRoots1[0], quadRoots1[1], quadRoots2[0], quadRoots2[1]};
    }

    /**
     * Get either 1 or 3 real root from the given cubic.
     *
     * @param a cubic coefficient, not equal to 0
     * @param b quadratic coefficient
     * @param c linear coefficient
     * @param d constant coefficient
     * @return an array of the real roots, of length 1 or 3
     */
    public static double[] cubicRealRoots(double a, double b, double c, double d) {
        double p = (3 * a * c - b * b) / (3 * a * a);
        double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);
        double delta = q * q / 4 + p * p * p / 27;
        if (delta > 0) {
            // Cardano's method
            // One real root
            double sqrtDelta = Math.sqrt(delta);
            double u = Math.cbrt(-q / 2 + sqrtDelta);
            double v = Math.cbrt(-q / 2 - sqrtDelta);
            return new double[] {u + v - b / (3 * a)};
        }
        // Trigonometric method
        // Three real roots
        double r = 2 * Math.sqrt(-p / 3);
        double phi = Math.acos(-4 * q / (r * r * r)) / 3;
        double[] roots = new double[3];
        for (int k = 0; k < 3; k++) {
            roots[k] = r * Math.cos(phi + k * 2 * Math.PI / 3) - b / (3 * a);
        }
        return roots;
    }

    /**
     * Get 0 or 2 real roots from the given quadratic.
     *
     * @param a quadratic coefficient, not equal to 0
     * @param b linear coefficient
     * @param c constant coefficient
     * @return an array of the real roots, of length 0 or 2
     */
    public static double[] quadraticRealRoots(double a, double b, double c) {
        double bp = b / 2;
        double delta = bp * bp - a * c;
        if (delta < 0) {
            return new double[] {};
        }
        double r1 = (-bp - Math.sqrt(delta)) / a;
        double r2 = -r1 - b / a;
        return new double[] {r1, r2};
    }
}
