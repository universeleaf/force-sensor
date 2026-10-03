"""Protect exact experiment identity across a MATLAB JSON resume."""
from copy import deepcopy
import unittest

from verify_publication_release import case_identity


class PublicationSerializationTest(unittest.TestCase):
    def test_singleton_and_missing_diagnostic_encodings(self):
        original = {"id": "fixture", "solver": {"stages": [{"objective": 1.5}],
                    "activeBranchPolish": [{"exitflag": 1}]},
                    "coverage": {"empiricalCoverage": None, "meanIntervalWidthN": None}}
        resumed = deepcopy(original)
        resumed["solver"]["stages"] = resumed["solver"]["stages"][0]
        resumed["solver"]["activeBranchPolish"] = resumed["solver"]["activeBranchPolish"][0]
        resumed["coverage"] = {"empiricalCoverage": [], "meanIntervalWidthN": []}
        self.assertEqual(case_identity(original), case_identity(resumed))
        self.assertIsNone(original["coverage"]["empiricalCoverage"])

    def test_numeric_tampering_still_fails(self):
        original = {"solver": {"stages": [{"objective": 1.5}]}}
        tampered = {"solver": {"stages": {"objective": 1.5000000000000002}}}
        self.assertNotEqual(case_identity(original), case_identity(tampered))

    def test_unrecognized_fields_and_order_are_not_normalized(self):
        self.assertNotEqual(case_identity({"unrecognized": None}),
                            case_identity({"unrecognized": []}))
        self.assertNotEqual(case_identity({"solver": {"stages": [{"id": 1}, {"id": 2}]}}),
                            case_identity({"solver": {"stages": [{"id": 2}, {"id": 1}]}}))


if __name__ == "__main__":
    unittest.main()
