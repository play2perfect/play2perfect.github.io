document.addEventListener('DOMContentLoaded', () => {
  const launch = document.getElementById('launch-interactive');
  const close = document.getElementById('close-interactive');
  const preview = document.getElementById('interactive-preview');
  const mount = document.getElementById('interactive-mount');
  launch.addEventListener('click', () => {
    if (mount.firstChild) return;
    document.querySelectorAll('video').forEach(video => video.pause());
    const frame = document.createElement('iframe');
    frame.src = './interactive/?embed=1';
    frame.title = 'Interactive Play2Perfect assembly simulation';
    frame.allowFullscreen = true;
    mount.appendChild(frame);
    preview.hidden = true;
    close.hidden = false;
  });
  close.addEventListener('click', () => {
    mount.replaceChildren(); // Removing the frame also stops its physics worker.
    preview.hidden = false;
    close.hidden = true;
    launch.focus();
  });
});
