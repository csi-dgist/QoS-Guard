// Make the header title text ("QoS Guard") link to the home page, matching the
// logo icon. Material only wraps the logo icon in the home link by default.
(function () {
  function wire() {
    var title = document.querySelector(".md-header__title");
    var logo = document.querySelector(".md-header__button.md-logo");
    if (!title || !logo) return;
    var href = logo.getAttribute("href");
    if (!href) return;
    title.style.cursor = "pointer";
    title.addEventListener("click", function (e) {
      // Ignore clicks on interactive children (e.g. the search box in the title).
      if (e.target.closest("input, button, .md-search")) return;
      window.location.href = new URL(href, window.location.href).href;
    });
  }
  if (document.readyState !== "loading") wire();
  else document.addEventListener("DOMContentLoaded", wire);
})();
