#!/usr/bin/env bash
# build_gallery.sh: Injects a GIF gallery into the social_orca.html page.

set -e

GIF_SOURCE_DIR="social_orca"
GALLERY_PAGE="social_orca.html"
TEMPLATE_PAGE="social_orca.template.html"

# --- SETUP ---
# 1. Create a template from the gallery page if it doesn't exist
if [ ! -f "$TEMPLATE_PAGE" ]; then
  echo "Creating template file from $GALLERY_PAGE..."
  cp "$GALLERY_PAGE" "$TEMPLATE_PAGE"
fi

# 2. Check if the source GIF directory exists
if [ ! -d "$GIF_SOURCE_DIR" ]; then
  echo "⚠️ Error: Source directory '$GIF_SOURCE_DIR' not found."
  exit 1
fi

# --- BUILD PROCESS ---
# 3. Generate the HTML for the gallery grid
echo "Generating gallery HTML for $(ls -1q $GIF_SOURCE_DIR/*.gif | wc -l) GIFs..."
GALLERY_HTML=""
# Loop through all GIFs in the folder
for gif in "$GIF_SOURCE_DIR"/*.gif; do
  filename=$(basename "$gif")
  # Use the 'video' tag for better performance, just like the nerfies template.
  # The src path must be relative to the final HTML page.
  GALLERY_HTML="${GALLERY_HTML}
    <div class='column is-one-quarter'>
      <div class='publication-video-container'>
        <video playsinline autoplay loop muted>
          <source src='${gif}' type='video/mp4'>
        </video>
      </div>
      <p class='has-text-centered'>${filename}</p>
    </div>"
done
# Wrap the generated tiles in the Bulma columns container
FULL_GALLERY_HTML="<div class='columns is-multiline is-centered'>${GALLERY_HTML}</div>"

# 4. Inject the generated gallery into the final HTML file
echo "Building final $GALLERY_PAGE..."
awk -v r="$FULL_GALLERY_HTML" '{gsub(/<!-- GIF_GALLERY_PLACEHOLDER -->/,r)}1' "$TEMPLATE_PAGE" > "$GALLERY_PAGE"

echo "✅ Gallery page built successfully! Open '$GALLERY_PAGE' to view."
