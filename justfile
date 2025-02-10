gui:
  (cd python/src && uv run watchgod gui.launch)
dgui:
  (cd python/src && ENVIRONMENT=dev uv run watchgod gui.launch)
