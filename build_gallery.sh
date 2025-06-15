#!/usr/bin/env bash
# build_gallery.sh (Final, Flexible Version)
# Creates a gallery page from any given folder of GIFs.
# Usage: ./build_gallery.sh <folder_name>

set -e

# 1. Get the gallery name from the first argument (e.g., "social_orca_doorway")
GALLERY_NAME=$1
if [ -z "$GALLERY_NAME" ]; then
  echo "⚠️ Error: Please provide the name of the GIF folder as an argument."
  echo "   Usage: ./build_gallery.sh social_orca_doorway"
  exit 1
fi

# 2. Define all file and folder names based on the argument
GIF_SOURCE_DIR="$GALLERY_NAME"
GALLERY_PAGE="${GALLERY_NAME}.html"
TEMPLATE_PAGE="${GALLERY_NAME}.template.html"

# --- SETUP ---
# 3. Create a template file if one doesn't exist for this gallery
if [ ! -f "$TEMPLATE_PAGE" ]; then
  echo "Creating new template file: $TEMPLATE_PAGE..."
  # Create a generic template page that automatically links to your custom styles
  cat <<EOT > "$TEMPLATE_PAGE"
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>MRNforSMG - ${GALLERY_NAME} Gallery</title>
  <link href="https://fonts.googleapis.com/css?family=Google+Sans|Noto+Sans|Castoro" rel="stylesheet">
  <link rel="stylesheet" href="./static/css/bulma.min.css">
  <link rel="stylesheet" href="./static/css/index.css">
  <!-- This line automatically links your custom gallery styles -->
  <link rel="stylesheet" href="./static/css/custom-gallery.css">
</head>
<body>
<nav class="navbar"><div class="navbar-menu"><div class="navbar-start" style="flex-grow: 1; justify-content: center;"><a class="navbar-item" href="index.html"><span>Home</span></a></div></div></nav>
<section class="section"><div class="container is-max-widescreen">
  <!-- The title is also generated automatically, replacing underscores with spaces -->
  <h2 class="title is-3 has-text-centered">${GALLERY_NAME//_/ } Simulation Gallery</h2>
  <div class="content"><!-- GIF_GALLERY_PLACEHOLDER --></div>
</div></section>
</body></html>
EOT
fi

if [ ! -d "$GIF_SOURCE_DIR" ]; then
  echo "⚠️ Error: Source directory '$GIF_SOURCE_DIR' not found."
  exit 1
fi

# --- BUILD PROCESS ---
echo "Generating gallery HTML for $(ls -1q "$GIF_SOURCE_DIR"/*.gif | wc -l) GIFs..."
GALLERY_HTML=""
# Loop through all GIFs, sorting them numerically (run_1, run_2, ..., run_10)
for gif in $(ls -v "$GIF_SOURCE_DIR"/*.gif); do
  filename=$(basename "$gif")
  # Use the standard <img> tag for correct GIF display
  GALLERY_HTML="${GALLERY_HTML}
    <div class='column is-one-quarter'>
      <div class='content'><img src='${gif}' alt='${filename}'></div>
      <p class='has-text-centered'>${filename}</p>
    </div>"
done
FULL_GALLERY_HTML="<div class='columns is-multiline is-centered'>${GALLERY_HTML}</div>"

# Inject the generated gallery into the final HTML page
echo "Building final page: $GALLERY_PAGE..."
awk -v r="$FULL_GALLERY_HTML" '{gsub(/<!-- GIF_GALLERY_PLACEHOLDER -->/,r)}1' "$TEMPLATE_PAGE" > "$GALLERY_PAGE"

echo "✅ Gallery page '$GALLERY_PAGE' built successfully!"
