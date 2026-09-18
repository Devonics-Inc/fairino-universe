import yaml


def _truthy(value: str) -> bool:
    return value.strip().lower() == "true"

def _is_string_true(value: str) -> bool:
    return value.strip().lower() == "true"



def _flatten(d, parent_key="", sep="_"):
    """Flattens nested dicts so existing flat defaults.get('key') calls keep working."""
    items = {}
    for k, v in d.items():
        # keep both the plain key (for leaf lookups) and prefixed key
        if isinstance(v, dict):
            items.update(_flatten(v, k, sep=sep))
        else:
            items[k] = v
    return items


def _load_config_file(path):
    """Loads a YAML file into a flat dict. Returns {} if it doesn't exist."""
    try:
        with open(path) as f:
            return _flatten(yaml.safe_load(f) or {})
    except OSError:
        print(f"[INFO] Config not found: {path}, using built-in defaults")
        return {}
