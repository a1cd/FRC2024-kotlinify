package frc.robot.util

import Jama.Matrix
import Jama.QRDecomposition
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow

// NOTE: This file is available at
// http://algs4.cs.princeton.edu/14analysis/PolynomialRegression.java.html
/**
 * The `PolynomialRegression` class performs a polynomial regression on an set of *N*
 * data points (*y<sub>i</sub>*, *x<sub>i</sub>*). That is, it fits a polynomial
 * *y* = <sub>0</sub> + <sub>1</sub> *x* + <sub>2</sub>
 * *x*<sup>2</sup> + ... + <sub>*d*</sub> *x*<sup>*d*</sup> (where
 * *y* is the response variable, *x* is the predictor variable, and the
 * <sub>*i*</sub> are the regression coefficients) that minimizes the sum of squared
 * residuals of the multiple regression model. It also computes associated the coefficient of
 * determination *R*<sup>2</sup>.
 *
 *
 * This implementation performs a QR-decomposition of the underlying Vandermonde matrix, so it is
 * neither the fastest nor the most numerically stable way to perform the polynomial regression.
 *
 * @author Robert Sedgewick
 * @author Kevin Wayne
 */
class PolynomialRegression @JvmOverloads constructor(
    x: DoubleArray, y: DoubleArray, // degree of the polynomial regression
    private val degree: Int, // name of the predictor variable
    private val variableName: String = "n"
) :
    Comparable<PolynomialRegression> {
    private val beta: Matrix // the polynomial regression coefficients
    private val sse: Double // sum of squares due to error
    private var sst: Double = 0.0 // total sum of squares

    /**
     * Performs a polynomial reggression on the data points `(y[i], x[i])`.
     *
     * @param x the values of the predictor variable
     * @param y the corresponding values of the response variable
     * @param degree the degree of the polynomial to fit
     * @param variableName the name of the predictor variable
     * @throws IllegalArgumentException if the lengths of the two arrays are not equal
     */
    /**
     * Performs a polynomial reggression on the data points `(y[i], x[i])`. Uses n as the name
     * of the predictor variable.
     *
     * @param x the values of the predictor variable
     * @param y the corresponding values of the response variable
     * @param degree the degree of the polynomial to fit
     * @throws IllegalArgumentException if the lengths of the two arrays are not equal
     */
    init {
        this.variableName = variableName

        val n: Int = x.size
        var qr: QRDecomposition? = null
        var matrixX: Matrix? = null

        // in case Vandermonde matrix does not have full rank, reduce degree until it
        // does
        while (true) {
            // build Vandermonde matrix

            val vandermonde: Array<DoubleArray> = Array(n) { DoubleArray(this.degree + 1) }
            for (i in 0 until n) {
                for (j in 0..this.degree) {
                    vandermonde[i][j] = x[i].pow(j.toDouble())
                }
            }
            matrixX = Matrix(vandermonde)

            // find least squares solution
            qr = QRDecomposition(matrixX)
            if (qr.isFullRank()) break

            // decrease degree and try again
            degree--
        }

        // create matrix from vector
        val matrixY: Matrix = Matrix(y, n)

        // linear regression coefficients
        beta = qr!!.solve(matrixY)

        // mean of y[] values
        var sum: Double = 0.0
        for (i in 0 until n) sum += y[i]
        val mean: Double = sum / n

        // total variation to be accounted for
        for (i in 0 until n) {
            val dev: Double = y[i] - mean
            sst += dev * dev
        }

        // variation not accounted for
        val residuals: Matrix = matrixX!!.times(beta).minus(matrixY)
        sse = residuals.norm2() * residuals.norm2()
    }

    /**
     * Returns the `j`th regression coefficient.
     *
     * @param j the index
     * @return the `j`th regression coefficient
     */
    fun beta(j: Int): Double {
        // to make -0.0 print as 0.0
        if (abs(beta.get(j, 0)) < 1E-4) return 0.0
        return beta.get(j, 0)
    }

    /**
     * Returns the degree of the polynomial to fit.
     *
     * @return the degree of the polynomial to fit
     */
    private fun degree(): Int {
        return degree
    }

    /**
     * Returns the coefficient of determination *R*<sup>2</sup>.
     *
     * @return the coefficient of determination *R*<sup>2</sup>, which is a real number between
     * 0 and 1
     */
    fun R2(): Double {
        if (sst == 0.0) return 1.0 // constant function

        return 1.0 - sse / sst
    }

    /**
     * Returns the expected response `y` given the value of the predictor variable `x`.
     *
     * @param x the value of the predictor variable
     * @return the expected response `y` given the value of the predictor variable `x`
     */
    fun predict(x: Double): Double {
        // horner's method
        var y: Double = 0.0
        for (j in degree downTo 0) y = beta(j) + (x * y)
        return y
    }

    /**
     * Returns a string representation of the polynomial regression model.
     *
     * @return a string representation of the polynomial regression model, including the best-fit
     * polynomial and the coefficient of determination *R*<sup>2</sup>
     */
    override fun toString(): String {
        var s: StringBuilder = StringBuilder()
        var j: Int = degree

        // ignoring leading zero coefficients
        while (j >= 0 && abs(beta(j)) < 1E-5) j--

        // create remaining terms
        while (j >= 0) {
            if (j == 0) s.append(String.format("%.10f ", beta(j)))
            else if (j == 1) s.append(String.format("%.10f %s + ", beta(j), variableName))
            else s.append(String.format("%.10f %s^%d + ", beta(j), variableName, j))
            j--
        }
        s = s.append("  (R^2 = " + String.format("%.3f", R2()) + ")")

        // replace "+ -2n" with "- 2n"
        return s.toString().replace("+ -", "- ")
    }

    /** Compare lexicographically.  */
    override fun compareTo(that: PolynomialRegression): Int {
        val EPSILON: Double = 1E-5
        val maxDegree: Int = max(degree().toDouble(), that.degree().toDouble()).toInt()
        for (j in maxDegree downTo 0) {
            var term1: Double = 0.0
            var term2: Double = 0.0
            if (this.degree() >= j) term1 = this.beta(j)
            if (that.degree() >= j) term2 = that.beta(j)
            if (abs(term1) < EPSILON) term1 = 0.0
            if (abs(term2) < EPSILON) term2 = 0.0
            if (term1 < term2) return -1
            else if (term1 > term2) return +1
        }
        return 0
    }

    companion object {
        /**
         * Unit tests the `PolynomialRegression` data type.
         *
         * @param args the command-line arguments
         */
        @JvmStatic
        fun main(args: Array<String>) {
            val x: DoubleArray = doubleArrayOf(10.0, 20.0, 40.0, 80.0, 160.0, 200.0)
            val y: DoubleArray = doubleArrayOf(100.0, 350.0, 1500.0, 6700.0, 20160.0, 40000.0)
            val regression: PolynomialRegression = PolynomialRegression(x, y, 3)

            println(regression)
        }
    }
}
