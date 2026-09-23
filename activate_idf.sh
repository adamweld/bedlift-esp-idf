# Activate ESP-IDF v5.5.1 (EIM install) in the current shell.
# Usage: source ./activate_idf.sh
_idf_activate="${IDF_ACTIVATE:-$HOME/.espressif/tools/activate_idf_v5.5.1.sh}"
if [ -f "$_idf_activate" ]; then
    . "$_idf_activate"
else
    echo "ESP-IDF activation script not found: $_idf_activate (see README Prerequisites)" >&2
fi
unset _idf_activate
