package frc.robot.util;

public final class PolynomialSolver {
    private PolynomialSolver() {}

    public static double[] depressedQuarticRealRoots(double a, double c, double d, double e) {
        /* Get 0, 2, or 4 real roots from the given depressed quartic
         * 
         * a != 0
        */

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

    public static double[] cubicRealRoots(double a, double b, double c, double d) {
        /* Get one real root from the given cubic. It always exists
         * 
         * If there are multiple, choose the on ewith the largest absolute value.
         */
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
        double r = Math.sqrt(-4 * p / 3);
        double phi = Math.acos(-q / (2 * p * Math.sqrt(-3 * p))) / 3;
        double[] roots = new double[3];
        for (int k = 0; k < 3; k++) {
            roots[k] = r * Math.cos(phi + k * 2 * Math.PI / 3) - b / (3 * a);
        }
        return roots;
    }

    public static double[] quadraticRealRoots(double a, double b, double c) {
        /* Get 0 or 2 real roots from the given quadratic
         * 
         * a != 0
         */

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
