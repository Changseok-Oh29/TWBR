"""Physical consistency checks for the supplied URDF and both base choices."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from twbr.modeling import compute_dynamics, load_model


URDF = Path(__file__).resolve().parents[1] / "2WL_V3_Assem_v2/urdf/2WL_V3_Assem_v2.urdf"


class ModelTests(unittest.TestCase):
    def test_mass_com_and_coordinates(self):
        xml = ET.parse(URDF).getroot()
        expected_mass = sum(float(e.attrib["value"]) for e in xml.findall("link/inertial/mass"))
        results = {}
        for base, nq, nv in (("fixed", 18, 12), ("floating", 25, 18)):
            with self.subTest(base=base):
                model = load_model(URDF, base=base)
                self.assertEqual((model.nq, model.nv), (nq, nv))
                q = pin.neutral(model)
                result = compute_dynamics(model, model.createData(), q, np.zeros(nv))
                self.assertAlmostEqual(result.total_mass, expected_mass, places=10)
                results[base] = result
        # Anchoring the same pose must not change whole-robot CoM or mass.
        np.testing.assert_allclose(
            results["fixed"].center_of_mass, results["floating"].center_of_mass, atol=1e-12,
        )
        self.assertAlmostEqual(results["floating"].gravity[2], expected_mass * 9.81, places=9)

    def test_dynamics_matches_inverse_dynamics(self):
        rng = np.random.default_rng(42)
        for base in ("fixed", "floating"):
            model = load_model(URDF, base=base)
            for sample in range(3):
                with self.subTest(base=base, sample=sample):
                    q = pin.integrate(model, pin.neutral(model), rng.uniform(-0.2, 0.2, model.nv))
                    for name in ("left_shoulder_joint", "right_shoulder_joint"):
                        q[model.joints[model.getJointId(name)].idx_q] = -0.3
                    v = rng.normal(0, 0.2, model.nv)
                    a = rng.normal(0, 0.2, model.nv)
                    result = compute_dynamics(model, model.createData(), q, v)
                    self.assertTrue(np.isfinite(result.mass_matrix).all())
                    self.assertGreater(np.linalg.eigvalsh(result.mass_matrix)[0], 0)
                    tau = pin.rnea(model, model.createData(), q, v, a)
                    np.testing.assert_allclose(
                        result.mass_matrix @ a + result.nonlinear_effects, tau,
                        rtol=1e-10, atol=1e-10,
                    )
                    static = compute_dynamics(model, model.createData(), q, np.zeros(model.nv))
                    np.testing.assert_allclose(static.nonlinear_effects, static.gravity, atol=1e-12)

    def test_rejects_invalid_configuration(self):
        model = load_model(URDF)
        with self.assertRaises(ValueError):
            compute_dynamics(model, model.createData(), np.zeros(model.nq), np.zeros(model.nv))


if __name__ == "__main__":
    unittest.main()
