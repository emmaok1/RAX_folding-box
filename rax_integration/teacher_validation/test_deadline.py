"""No processes, simulator or network: absolute cumulative task deadline guards."""
import unittest
from rax_integration.teacher.run import effective_wall_budget

class DeadlineChecks(unittest.TestCase):
    def test_continuation_uses_remaining_absolute_time(self):
        self.assertEqual(effective_wall_budget({'max_wall_seconds':124}, {'original_deadline_unix_s':1100}, now=1010), 90)
    def test_shorter_local_cap_is_retained(self):
        self.assertEqual(effective_wall_budget({'max_wall_seconds':20}, {'original_deadline_unix_s':1100}, now=1010), 20)
    def test_expired_or_invalid_deadline_refuses(self):
        for deadline in [1000, 999, float('inf'), float('nan'), True, '1100']:
            with self.subTest(deadline=deadline), self.assertRaises(PermissionError):
                effective_wall_budget({'max_wall_seconds':124}, {'original_deadline_unix_s':deadline}, now=1000)
    def test_ordinary_session_keeps_its_cap(self):
        self.assertEqual(effective_wall_budget({'max_wall_seconds':600}, {}, now=1000), 600)
