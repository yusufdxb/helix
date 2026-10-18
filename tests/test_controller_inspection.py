"""The finite inspector's RPC allowlist is a hardware boundary."""
import pytest

from scripts.hw.controller_inspect import QUERIES, metadata, query_payload


def test_query_allowlist_never_commands_sport_motion_or_changes_services():
    assert {(service, api) for _, service, api in QUERIES} == {
        ('sport', 1), ('robot_state', 1), ('motion_switcher', 1),
        ('robot_state', 1003), ('motion_switcher', 1001),
    }
    for name, service, api in QUERIES:
        assert query_payload(name) == (service, api, '{}')
    for name in ('Move', 'StopMove', 'Damp', 'ServiceSwitch', 'SelectMode', 'ReleaseMode'):
        with pytest.raises(StopIteration):
            query_payload(name)


def test_server_api_version_is_not_promoted_to_firmware_identity():
    result = metadata('sport_api_version', '1.0.0.1')
    assert result == {'server_api_version': '1.0.0.1', 'firmware_build': None}


def test_inspection_discards_undocumented_private_response_fields():
    assert metadata('check_mode', '{"name":"mcf","form":"0","serial":"private"}') == {
        'name': 'mcf', 'form': '0'}
    assert metadata('service_list', '[{"name":"sport_mode","status":1,"protect":1,"serial":"private"}]') == [
        {'name': 'sport_mode', 'status': 1, 'protect': 1}]
    with pytest.raises(ValueError):
        metadata('service_list', '{}')


@pytest.mark.parametrize('name,payload', [
    ('check_mode', '{}'),
    ('check_mode', '{"name":{"private":"value"},"form":"0"}'),
    ('service_list', '[{"name":"sport_mode","status":1}]'),
    ('service_list', '[{"name":"sport_mode","status":1,"protect":{"private":"value"}}]'),
])
def test_unknown_metadata_schema_is_a_failure_not_a_successful_empty_report(name, payload):
    with pytest.raises(ValueError, match='unexpected metadata schema'):
        metadata(name, payload)
