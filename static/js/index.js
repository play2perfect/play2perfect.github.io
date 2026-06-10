function switchAssemblyVideo(btn) {
  var buttons = document.querySelectorAll(".assembly-btn");
  buttons.forEach(function (button) {
    button.classList.remove("active");
  });
  btn.classList.add("active");

  var video = document.getElementById("assembly-task-video");
  var source = document.getElementById("assembly-task-source");
  var newSrc = btn.getAttribute("data-video");
  if (!video || !source || !newSrc) return;

  video.classList.add("switching");
  setTimeout(function () {
    source.setAttribute("src", newSrc);
    video.load();
    video.play().catch(function () {});
    video.classList.remove("switching");
  }, 250);
}

document.addEventListener("DOMContentLoaded", function () {
  function setupSoundOverlay(containerSelector, videoId, overlayId) {
    var container = document.querySelector(containerSelector);
    var video = document.getElementById(videoId);
    var overlay = document.getElementById(overlayId);
    if (!container || !video || !overlay) return;

    var overlayDismissed = false;
    var overlayTimer = null;

    function dismissOverlay(unmute) {
      if (overlayDismissed) return;
      overlayDismissed = true;

      if (overlayTimer) {
        clearTimeout(overlayTimer);
        overlayTimer = null;
      }

      if (unmute) {
        video.muted = false;
      }

      container.style.pointerEvents = "none";
      setTimeout(function () {
        container.style.pointerEvents = "";
      }, 400);

      video.play().then(function () {
        overlay.classList.add("fade-out");
      }).catch(function () {
        video.muted = true;
        video.play().then(function () {
          overlay.classList.add("fade-out");
        }).catch(function () {});
      });
    }

    if ("IntersectionObserver" in window) {
      var showcaseObserver = new IntersectionObserver(function (entries) {
        entries.forEach(function (entry) {
          if (entry.isIntersecting && !overlayDismissed) {
            overlayTimer = setTimeout(function () {
              dismissOverlay(false);
            }, 3000);
            showcaseObserver.unobserve(video);
          }
        });
      }, { threshold: 0.25 });

      showcaseObserver.observe(video);
    }

    overlay.addEventListener("click", function () {
      dismissOverlay(true);
    });

    overlay.addEventListener("touchend", function (event) {
      event.preventDefault();
      dismissOverlay(true);
    });
  }

  setupSoundOverlay(".showcase-video", "showcase-video", "sound-overlay");
  setupSoundOverlay(".recovery-video-container", "recovery", "recovery-sound-overlay");

  var lazyVideos = document.querySelectorAll("video.lazy-video");

  if ("IntersectionObserver" in window) {
    var videoObserver = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        var lazyVideo = entry.target;
        if (entry.isIntersecting) {
          lazyVideo.play().catch(function () {});
        } else {
          lazyVideo.pause();
        }
      });
    }, { threshold: 0.25 });

    lazyVideos.forEach(function (lazyVideo) {
      videoObserver.observe(lazyVideo);
    });
  } else {
    lazyVideos.forEach(function (lazyVideo) {
      lazyVideo.play().catch(function () {});
    });
  }
});
