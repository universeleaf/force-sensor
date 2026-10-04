"""Protect exact experiment identity and repeatable manuscript updates."""
from copy import deepcopy
import unittest
from unittest.mock import patch

import verify_publication_release as release
from verify_publication_release import case_identity
from update_publication_manuscript import replace_results_section


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


class PublicationManuscriptTest(unittest.TestCase):
    def test_repeated_updates_preserve_unrelated_content_and_unique_figures(self):
        prefix = "Introduction and methods stay intact.\n\n"
        prefix += "\\begin{figure*}\n\\label{fig:geometry}\n\\end{figure*}\n\n"
        suffix = "% Author, affiliation\n\\end{document}\n"
        tail = ("\\section{Simulation and Comparison Protocols}\n"
                "\\subsection{Factors, scoring, and provenance}\n"
                "\\subsection{Multicontact force separation}\n"
                "\\subsection{Factor ablations and local uncertainty}\n")
        for label in ("paired", "factors", "magnitude", "uncertainty"):
            tail += "\\begin{figure*}\n\\label{fig:" + label + "}\n\\end{figure*}\n\n"
        first = replace_results_section(prefix + tail + suffix, tail)
        second = replace_results_section(first, tail)
        self.assertEqual(first, second)
        self.assertTrue(second.startswith(prefix))
        self.assertTrue(second.endswith(suffix))
        for label in ("geometry", "paired", "factors", "magnitude", "uncertainty"):
            self.assertEqual(second.count("\\label{fig:" + label + "}"), 1)

    def test_missing_section_marker_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "section marker is not unique"):
            replace_results_section("Unrelated manuscript.\n", "Replacement.\n")


class PublicationCompletionTest(unittest.TestCase):
    """Integration checks use the published evidence without changing files."""

    def assert_bad_metadata_rejected(self, suffix, mutate, message):
        original_read = release.read

        def read_fixture(path):
            report = deepcopy(original_read(path))
            if path.as_posix().endswith(suffix):
                mutate(report)
            return report

        with patch.object(release, "read", side_effect=read_fixture):
            with self.assertRaisesRegex(ValueError, message):
                release.verify()

    def test_duplicate_completion_step_is_rejected(self):
        self.assert_bad_metadata_rejected("publication/completion.json",
            lambda r: r["completedSteps"].append(deepcopy(r["completedSteps"][0])),
            "Four distinct completed steps")

    def test_unfinished_baseline_is_rejected(self):
        self.assert_bad_metadata_rejected("formulation_literature/comparison.json",
            lambda r: r["cases"][0].update(completed=False), "Incomplete inference ledger")

    def test_empty_engineering_checks_are_rejected(self):
        self.assert_bad_metadata_rejected("completion/project_checks.json",
            lambda r: r.update(checks=[]), "Engineering checks failed")


if __name__ == "__main__":
    unittest.main()
