gui:
  (cd python/src && uv run watchgod train.main)
dgui:
  (cd python/src && DEV=true uv run train.py)
dwgui:
  (cd python/src && DEV=true uv run watchgod train.main)
