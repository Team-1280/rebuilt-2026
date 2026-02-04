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
        // Choose root with largest absolute value
        double largestAbs = 0;
        for (double i : cubicRealRoots(A, B, C, D)) {
            if (Math.abs(i) > largestAbs) {
                largestAbs = i;
            }
        }
        double y1 = largestAbs;
        // Solve the two quadratics
        double u = Math.sqrt(2 * y1 - p);
        double v = Math.sqrt(y1 * y1 - r);
        double[] quadRoots1 = quadraticRealRoots(1.0, -u, y1 - v);
        double[] quadRoots2 = quadraticRealRoots(1.0, u, y1 - v);
        // add together the solutions
        double[] sumPart1 = new double[2];
        double[] sumPart2 = new double[2];
        double[] roots = new double[2];
        switch (quadRoots1.length) {
            case 0:
                sumPart1[0] = 0;
                sumPart1[1] = 0;
                break;
            case 1:
                sumPart1[0] = quadRoots1[0];
                sumPart1[1] = 0;
                break;
            case 2:
                sumPart1 = quadRoots1;
        }
        switch (quadRoots2.length) {
            case 0:
                sumPart2[0] = 0;
                sumPart2[1] = 0;
                break;
            case 1:
                sumPart2[0] = quadRoots2[0];
                sumPart2[1] = 0;
                break;
            case 2:
                sumPart2 = quadRoots2;
        }
        for (int i = 0; i < 2; i++) {
            roots[i] = sumPart1[i] + sumPart2[i];
        }
        return roots;
    }

    public static double[] cubicRealRoots(double a, double b, double c, double d) {
        /* Get one real root from the given cubic. It always exists
         * 
         * If there are multiple, choose the on ewith the largest absolute value.
         */
        double p = (3 * a * c - b * b) / (3 * a * a);
        double q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);
        double delta = Math.pow((q / 2), 2) + Math.pow((p / 3), 3);
        if (delta > 0) {
            // Cardano's method
            // One real root
            double sqrt_delta = Math.sqrt(delta);
            double u = Math.cbrt(-q / 2 + sqrt_delta);
            double v = Math.cbrt(-q / 2 - sqrt_delta);
            return new double[] {u + v - b / (3 * a)};
        }
        // Trigonometric method
        // Three real roots
        double r = Math.sqrt(-4 * p / 3);
        double phi = Math.acos(-q / 2 / Math.sqrt(-(Math.pow((p / 3), 3)))) / 3;
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
