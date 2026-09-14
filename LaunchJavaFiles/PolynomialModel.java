import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;

/**
 * Loads and evaluates a 2-variable polynomial model from a JSON file.
 *
 * <p>Expected JSON format: { "intercept": double, "coefs": [c0, c1, ..., cN], "powers": [[p0_a,
 * p0_b], [p1_a, p1_b], ..., [pN_a, pN_b]] }
 *
 * <p>Evaluation: intercept + sum_i( coefs[i] * x^powers[i][0] * y^powers[i][1] )
 */
public class PolynomialModel {

    private final double intercept;
    private final double[] coefs;
    private final int[][] powers;

    private PolynomialModel(double intercept, double[] coefs, int[][] powers) {
        this.intercept = intercept;
        this.coefs = coefs;
        this.powers = powers;
    }

    /**
     * Loads a PolynomialModel from the given JSON file path.
     *
     * @param path path to the model JSON file
     * @return loaded PolynomialModel
     * @throws IOException if the file cannot be read or parsed
     */
    public static PolynomialModel load(String path) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        RawModel raw = mapper.readValue(new File(path), RawModel.class);
        return new PolynomialModel(raw.intercept, raw.coefs, raw.powers);
    }

    /**
     * Evaluates the polynomial at (x, y).
     *
     * @param x first variable (e.g. distance r)
     * @param y second variable (e.g. velocity vf or vl)
     * @return polynomial value
     */
    public double evaluate(double x, double y) {
        double result = intercept;
        for (int i = 0; i < coefs.length; i++) {
            if (coefs[i] == 0.0) continue;
            result += coefs[i] * Math.pow(x, powers[i][0]) * Math.pow(y, powers[i][1]);
        }
        return result;
    }

    // Jackson deserialization target

    private static class RawModel {
        public double intercept;
        public double[] coefs;
        public int[][] powers;
    }
}
